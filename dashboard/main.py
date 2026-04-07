"""
FireVisonBot — Dashboard: đọc MJPEG từ CAM, YOLO trên laptop, /ai_feed + telemetry S3.

Chạy từ thư mục dashboard/:
  pip install -r requirements.txt
  uvicorn main:app --host 0.0.0.0 --port 8765

Mở trình duyệt: http://localhost:8765
Model mặc định: <repo>/models/yolo_best.pt
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import cv2
import httpx
import numpy as np
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from ultralytics import YOLO

from stream_reader import MjpegStreamReader

BASE_DIR = Path(__file__).resolve().parent
REPO_ROOT = BASE_DIR.parent
DEFAULT_MODEL = REPO_ROOT / "models" / "yolo_best.pt"
CONFIG_PATH = REPO_ROOT / "config" / "system_config.json"


class RuntimeConfig:
    stream_url: str = "http://192.168.137.108:81/stream"
    # Base URL ESP32-CAM để bật/tắt stream theo sự kiện lửa.
    cam_control_base_url: str = "http://192.168.137.108:81"
    # Nếu false: khi /camera/on lỗi vẫn thử đọc /stream (hỗ trợ firmware CAM cũ chưa có API control).
    cam_control_strict: bool = False
    # IP ESP32-S3 (đổi khi DHCP cấp IP khác). Không dùng .1 (gateway laptop).
    robot_status_url: str = "http://192.168.137.71:8080/api/status"
    yolo_imgsz: int = 416
    yolo_conf: float = 0.45
    infer_interval_s: float = 0.12
    robot_poll_s: float = 0.4
    # Timeout HTTP tới ESP32-S3 (s). Quá ngắn dễ thấy "timeout" dù IP đúng.
    robot_http_timeout_s: float = 12.0
    # Sau khi không còn tín hiệu lửa từ S3 trong khoảng này thì tắt camera để tiết kiệm pin.
    cam_auto_off_delay_s: float = 3.5
    ai_feed_jpeg_quality: int = 95  # /ai_feed MJPEG (60–98, cao = nét hơn trên dashboard)
    # Đẩy độ tin cậy YOLO (0…1) lên ESP32-S3 để bật relay khi kết hợp IR trái + >= 65%.
    robot_ai_push_url: str = "http://192.168.137.71:8080/api/ai_fire"
    # Tùy chọn: nếu khác rỗng, ESP32-CAM phải nối ws://host:port/ws/cam?token=...
    cam_ws_token: str = ""


cfg = RuntimeConfig()


def _cfg_to_dict() -> dict[str, Any]:
    return {
        "stream_url": cfg.stream_url,
        "robot_status_url": cfg.robot_status_url,
        "cam_control_base_url": cfg.cam_control_base_url,
        "cam_control_strict": cfg.cam_control_strict,
        "yolo_imgsz": cfg.yolo_imgsz,
        "yolo_conf": cfg.yolo_conf,
        "infer_interval_s": cfg.infer_interval_s,
        "robot_poll_s": cfg.robot_poll_s,
        "robot_http_timeout_s": cfg.robot_http_timeout_s,
        "cam_auto_off_delay_s": cfg.cam_auto_off_delay_s,
        "ai_feed_jpeg_quality": cfg.ai_feed_jpeg_quality,
        "robot_ai_push_url": cfg.robot_ai_push_url,
        "cam_ws_token": cfg.cam_ws_token,
    }


def _cfg_apply_dict(data: dict[str, Any]) -> None:
    if "stream_url" in data:
        cfg.stream_url = str(data["stream_url"])
    if "robot_status_url" in data:
        cfg.robot_status_url = str(data["robot_status_url"])
    if "cam_control_base_url" in data:
        cfg.cam_control_base_url = str(data["cam_control_base_url"])
    if "cam_control_strict" in data:
        cfg.cam_control_strict = bool(data["cam_control_strict"])
    if "yolo_imgsz" in data:
        cfg.yolo_imgsz = int(data["yolo_imgsz"])
    if "yolo_conf" in data:
        cfg.yolo_conf = float(data["yolo_conf"])
    if "infer_interval_s" in data:
        cfg.infer_interval_s = float(data["infer_interval_s"])
    if "robot_poll_s" in data:
        cfg.robot_poll_s = float(data["robot_poll_s"])
    if "robot_http_timeout_s" in data:
        cfg.robot_http_timeout_s = max(3.0, float(data["robot_http_timeout_s"]))
    if "cam_auto_off_delay_s" in data:
        cfg.cam_auto_off_delay_s = max(1.0, float(data["cam_auto_off_delay_s"]))
    if "ai_feed_jpeg_quality" in data:
        q = int(data["ai_feed_jpeg_quality"])
        cfg.ai_feed_jpeg_quality = max(60, min(98, q))
    if "robot_ai_push_url" in data:
        cfg.robot_ai_push_url = str(data["robot_ai_push_url"])
    if "cam_ws_token" in data:
        cfg.cam_ws_token = str(data["cam_ws_token"])


def _load_config_file() -> None:
    if not CONFIG_PATH.is_file():
        return
    try:
        data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            _cfg_apply_dict(data)
            _log(f"Đã nạp cấu hình: {CONFIG_PATH}")
    except Exception as exc:
        _log(f"CẢNH BÁO: không đọc được file cấu hình {CONFIG_PATH}: {exc}")


def _save_config_file() -> None:
    try:
        CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        CONFIG_PATH.write_text(
            json.dumps(_cfg_to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except Exception as exc:
        _log(f"CẢNH BÁO: không ghi được file cấu hình {CONFIG_PATH}: {exc}")


def _esp_http_timeout() -> httpx.Timeout:
    # Tránh “kẹt” lâu khi Wi‑Fi chập chờn: connect/pool nhanh, read dùng theo cấu hình.
    t = max(3.0, float(cfg.robot_http_timeout_s))
    return httpx.Timeout(connect=2.0, read=t, write=2.0, pool=2.0)


def _fmt_robot_poll_error(exc: BaseException) -> str:
    base = str(exc)
    low = base.lower()
    if "timed out" in low or "timeout" in low:
        return (
            base
            + " — Gợi ý: (1) Trên PC: ping IP S3; (2) Serial S3 có dòng "
            "'Telemetry: GET /api/status ... :8080' sau khi có WiFi? "
            "(3) URL đúng http://IP:8080/api/status (4) Firewall không chặn cổng 8080."
        )
    if "connection refused" in low or "actively refused" in low:
        return base + " — HTTP server S3 chưa chạy (chưa WiFi / chưa tới bước khởi động telemetry)."
    if "no route to host" in low or "host unreachable" in low:
        return base + " — PC và ESP32-S3 không cùng mạng hoặc sai IP."
    return base


stream_reader: MjpegStreamReader | None = None
stream_lock = threading.Lock()
cam_stream_active = False
last_fire_seen_ts = 0.0
model: YOLO | None = None
annotated_lock = threading.Lock()
annotated_frame: np.ndarray | None = None
inference_meta: dict[str, Any] = {}
worker_stop = threading.Event()
main_loop: asyncio.AbstractEventLoop | None = None

log_lines: list[str] = []
log_lock = threading.Lock()
MAX_LOG = 400

inf_thread: threading.Thread | None = None
robot_thread: threading.Thread | None = None


def _log(line: str) -> None:
    ts = time.strftime("%H:%M:%S")
    s = f"[{ts}] {line}"
    print(s)
    with log_lock:
        log_lines.append(s)
        while len(log_lines) > MAX_LOG:
            log_lines.pop(0)
    if main_loop and main_loop.is_running():
        try:
            asyncio.run_coroutine_threadsafe(hub.broadcast({"type": "log", "line": s}), main_loop)
        except RuntimeError:
            pass


class ConnectionHub:
    def __init__(self):
        self._clients: list[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self._clients.append(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self._clients:
            self._clients.remove(ws)

    async def broadcast(self, msg: dict):
        dead = []
        text = json.dumps(msg, ensure_ascii=False)
        for c in self._clients:
            try:
                await c.send_text(text)
            except Exception:
                dead.append(c)
        for c in dead:
            self.disconnect(c)


hub = ConnectionHub()


class CamCommandHub:
    """Một hoặc nhiều ESP32-CAM nối WebSocket vào đây; nhận JSON {\"cmd\":\"stream_on|stream_off\"}."""

    def __init__(self):
        self._clients: list[WebSocket] = []
        self._lock = asyncio.Lock()

    async def register(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.append(ws)
            n = len(self._clients)
        _log(
            f"ESP32-CAM WebSocket: đã kết nối (tổng {n}) — chờ lệnh stream_on/stream_off"
        )

    async def unregister(self, ws: WebSocket) -> None:
        async with self._lock:
            if ws in self._clients:
                self._clients.remove(ws)

    async def broadcast_cmd(self, cmd: str) -> int:
        msg = json.dumps({"cmd": cmd}, ensure_ascii=False)
        async with self._lock:
            clients = list(self._clients)
        sent = 0
        dead: list[WebSocket] = []
        for c in clients:
            try:
                await c.send_text(msg)
                sent += 1
            except Exception:
                dead.append(c)
        if dead:
            async with self._lock:
                for d in dead:
                    if d in self._clients:
                        self._clients.remove(d)
        return sent


cam_hub = CamCommandHub()


def _notify_cam_cmd(cmd: str) -> int:
    """Gửi lệnh tới mọi CAM đang nối WS (gọi từ thread worker)."""
    if main_loop is None:
        return 0
    try:
        fut = asyncio.run_coroutine_threadsafe(cam_hub.broadcast_cmd(cmd), main_loop)
        return int(fut.result(timeout=3.0))
    except Exception as exc:
        _log(f"Bắn lệnh WebSocket camera lỗi: {exc}")
        return 0


def restart_stream_reader() -> None:
    global stream_reader
    with stream_lock:
        if stream_reader is not None:
            stream_reader.stop()
            stream_reader = None
        time.sleep(0.15)
        url = (cfg.stream_url or "").strip()
        if url:
            stream_reader = MjpegStreamReader(url).start()
            _log(f"MJPEG stream: {url}")
        else:
            _log("MJPEG stream: (trống — cấu hình URL trong UI)")


def _cam_control_url(path: str) -> str:
    base = (cfg.cam_control_base_url or "").strip().rstrip("/")
    return f"{base}{path}"


def _set_cam_stream_active(enable: bool) -> None:
    global stream_reader, cam_stream_active
    if enable == cam_stream_active:
        return

    cmd = "stream_on" if enable else "stream_off"
    n_ws = _notify_cam_cmd(cmd)
    if n_ws:
        _log(f"Lệnh camera qua WebSocket → {n_ws} client")

    # Cho firmware kịp bật/tắt sensor trước khi laptop mở MJPEG.
    time.sleep(0.12)

    control_ok = n_ws > 0
    base = (cfg.cam_control_base_url or "").strip()

    if not control_ok and base:
        path = "/camera/on" if enable else "/camera/off"
        try:
            with httpx.Client(timeout=_esp_http_timeout()) as client:
                r = client.post(_cam_control_url(path))
                r.raise_for_status()
            control_ok = True
            _log(f"Lệnh camera qua HTTP {path} OK")
        except Exception as exc:
            _log(f"Lỗi gửi lệnh camera {path}: {exc}")
            if cfg.cam_control_strict:
                return
            _log(
                "Fallback: vẫn tiếp tục bật/tắt stream trên laptop (giả sử CAM firmware cũ luôn stream)."
            )
    elif not control_ok and not base:
        _log(
            "CẢNH BÁO: không có client WebSocket camera và cam_control_base_url trống — không gửi lệnh tới CAM."
        )
        if cfg.cam_control_strict:
            return

    with stream_lock:
        if stream_reader is not None:
            stream_reader.stop()
            stream_reader = None
        if enable:
            time.sleep(0.15)
            su = (cfg.stream_url or "").strip()
            if su:
                stream_reader = MjpegStreamReader(su).start()
            else:
                _log("stream_url trống — không mở MJPEG reader")
    cam_stream_active = enable
    if enable:
        _log("ESP32-CAM: STREAM ON" if control_ok else "ESP32-CAM: STREAM ON (fallback)")
    else:
        _log("ESP32-CAM: STREAM OFF" if control_ok else "ESP32-CAM: STREAM OFF (fallback)")


_ai_push_err_log_ts: float = 0.0


def _push_robot_ai_confidence(confidence: float) -> None:
    """Gửi độ tin cậy YOLO lên ESP32-S3 (POST /api/ai_fire) — không phải ESP gọi backend."""
    global _ai_push_err_log_ts
    url = (cfg.robot_ai_push_url or "").strip()
    if not url:
        return
    try:
        with httpx.Client(timeout=_esp_http_timeout()) as client:
            r = client.post(url, json={"confidence": float(confidence)})
            if r.status_code != 200:
                now = time.monotonic()
                if now - _ai_push_err_log_ts >= 5.0:
                    _ai_push_err_log_ts = now
                    _log(
                        f"POST ai_fire HTTP {r.status_code} (ESP có nhận không?): {r.text[:120]!r}"
                    )
    except Exception as exc:
        now = time.monotonic()
        if now - _ai_push_err_log_ts >= 5.0:
            _ai_push_err_log_ts = now
            _log(
                f"POST ai_fire lỗi mạng → ESP32-S3 không cập nhật confidence: {exc!r} | URL={url!r}"
            )


def _inference_worker():
    global annotated_frame, inference_meta
    _log("Worker AI: chạy.")
    while not worker_stop.is_set():
        if model is None:
            time.sleep(0.5)
            continue
        with stream_lock:
            sr = stream_reader
        if sr is None:
            time.sleep(0.2)
            continue
        t0 = time.perf_counter()
        frame = sr.read()
        if frame is None:
            time.sleep(0.05)
            continue
        try:
            results = model.predict(
                source=frame,
                imgsz=cfg.yolo_imgsz,
                conf=cfg.yolo_conf,
                verbose=False,
            )
            plotted = results[0].plot()
            names = getattr(results[0], "names", None) or {}
            boxes = []
            r0 = results[0]
            if r0.boxes is not None and len(r0.boxes):
                for b in r0.boxes:
                    cid = int(b.cls[0]) if b.cls is not None else -1
                    conf = float(b.conf[0]) if b.conf is not None else 0.0
                    xyxy = b.xyxy[0].tolist() if b.xyxy is not None else []
                    boxes.append(
                        {
                            "cls": cid,
                            "name": names.get(cid, str(cid)),
                            "conf": round(conf, 3),
                            "xyxy": [round(x, 1) for x in xyxy],
                        }
                    )
            latency_ms = (time.perf_counter() - t0) * 1000
            max_conf = max((float(b["conf"]) for b in boxes), default=0.0)
            _push_robot_ai_confidence(max_conf)

            meta = {
                "latency_ms": round(latency_ms, 1),
                "num_detections": len(boxes),
                "boxes": boxes[:32],
                "max_conf": round(max_conf, 4),
            }
            with annotated_lock:
                annotated_frame = plotted
                inference_meta = meta
            if main_loop and main_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    hub.broadcast({"type": "detection", "data": meta}),
                    main_loop,
                )
        except Exception as exc:
            _log(f"YOLO lỗi: {exc}")
        time.sleep(cfg.infer_interval_s)


def _robot_poll_worker():
    global last_fire_seen_ts
    while not worker_stop.is_set():
        url = (cfg.robot_status_url or "").strip()
        if not url:
            if main_loop and main_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    hub.broadcast(
                        {
                            "type": "sensors",
                            "data": {
                                "_ok": False,
                                "error": "Chưa cấu hình URL telemetry ESP32-S3 (IP thật của S3, :8080/api/status)",
                            },
                        }
                    ),
                    main_loop,
                )
            time.sleep(2.5)
            continue
        try:
            with httpx.Client(timeout=_esp_http_timeout()) as client:
                r = client.get(url)
                r.raise_for_status()
                data = r.json()
                data["_ok"] = True
                # CRITICAL: broadcast telemetry NGAY để UI không bị "trễ/đảo" khi phần bật/tắt camera
                # bị timeout (HTTP /camera/on|off). Camera control không được phép block đường IR.
                if main_loop and main_loop.is_running():
                    asyncio.run_coroutine_threadsafe(
                        hub.broadcast({"type": "sensors", "data": data}),
                        main_loop,
                    )

                fire_any = bool(data.get("flame_left")) or bool(data.get("flame_right"))
                now = time.monotonic()
                if fire_any:
                    last_fire_seen_ts = now
                    if not cam_stream_active:
                        _set_cam_stream_active(True)
                elif cam_stream_active and (now - last_fire_seen_ts) >= cfg.cam_auto_off_delay_s:
                    _set_cam_stream_active(False)
        except Exception as exc:
            payload = {"_ok": False, "error": _fmt_robot_poll_error(exc)}
            if main_loop and main_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    hub.broadcast({"type": "sensors", "data": payload}),
                    main_loop,
                )
        time.sleep(cfg.robot_poll_s)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global main_loop, model, inf_thread, robot_thread

    main_loop = asyncio.get_running_loop()
    worker_stop.clear()
    _load_config_file()

    if DEFAULT_MODEL.is_file():
        _log(f"Đang tải YOLO: {DEFAULT_MODEL}")
        model = YOLO(str(DEFAULT_MODEL))
        _log("YOLO sẵn sàng.")
    else:
        _log(f"CẢNH BÁO: Không thấy model tại {DEFAULT_MODEL}")

    inf_thread = threading.Thread(target=_inference_worker, daemon=True)
    robot_thread = threading.Thread(target=_robot_poll_worker, daemon=True)
    inf_thread.start()
    robot_thread.start()
    # Khởi động ở trạng thái nghỉ: camera tắt, chỉ bật khi S3 báo có lửa.
    _set_cam_stream_active(False)

    yield

    worker_stop.set()
    global stream_reader
    with stream_lock:
        if stream_reader is not None:
            stream_reader.stop()
            stream_reader = None
    _set_cam_stream_active(False)


app = FastAPI(title="FireVisonBot Dashboard", lifespan=lifespan)
templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")


def _mjpeg_bytes(frame: np.ndarray) -> bytes:
    # Chất lượng cao cho /ai_feed (82 dễ thấy răng cưa khi phóng to trong browser)
    params = [
        int(cv2.IMWRITE_JPEG_QUALITY),
        cfg.ai_feed_jpeg_quality,
        int(cv2.IMWRITE_JPEG_OPTIMIZE),
        1,
    ]
    ok, buf = cv2.imencode(".jpg", frame, params)
    if not ok:
        return b""
    return buf.tobytes()


async def ai_feed():
    boundary = b"frame"
    while True:
        with annotated_lock:
            frame = None if annotated_frame is None else annotated_frame.copy()
        if frame is None:
            await asyncio.sleep(0.05)
            continue
        jpg = await asyncio.to_thread(_mjpeg_bytes, frame)
        if not jpg:
            await asyncio.sleep(0.02)
            continue
        yield (
            b"--"
            + boundary
            + b"\r\nContent-Type: image/jpeg\r\n\r\n"
            + jpg
            + b"\r\n"
        )
        await asyncio.sleep(0.04)


@app.get("/ai_feed")
async def ai_feed_route():
    return StreamingResponse(ai_feed(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/api/config")
async def api_config_get():
    return {
        **_cfg_to_dict(),
        "robot_http_timeout_s": cfg.robot_http_timeout_s,
        "model_path": str(DEFAULT_MODEL),
        "model_exists": DEFAULT_MODEL.is_file(),
    }


@app.post("/api/config")
async def api_config_post(request: Request):
    body = await request.json()
    _cfg_apply_dict(body)
    _save_config_file()
    if cam_stream_active:
        await asyncio.to_thread(restart_stream_reader)
    _log("Đã cập nhật cấu hình (stream/robot/YOLO).")
    return await api_config_get()


@app.get("/api/robot")
async def api_robot_proxy():
    url = (cfg.robot_status_url or "").strip()
    if not url:
        return {"_ok": False, "error": "robot_status_url trống"}
    try:
        async with httpx.AsyncClient(timeout=_esp_http_timeout()) as client:
            r = await client.get(url)
            r.raise_for_status()
            data = r.json()
            data["_ok"] = True
            return data
    except Exception as exc:
        return {"_ok": False, "error": _fmt_robot_poll_error(exc)}


@app.get("/api/logs")
async def api_logs():
    with log_lock:
        return {"lines": list(log_lines[-200:])}


@app.get("/api/inference")
async def api_inference():
    with annotated_lock:
        return dict(inference_meta)


@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "default_robot_url": cfg.robot_status_url,
            "model_path": str(DEFAULT_MODEL),
        },
    )


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await hub.connect(ws)
    try:
        with log_lock:
            boot = {"type": "log_bulk", "lines": list(log_lines[-80:])}
        await ws.send_text(json.dumps(boot, ensure_ascii=False))
        while True:
            await ws.receive()
    except WebSocketDisconnect:
        pass
    finally:
        hub.disconnect(ws)


@app.websocket("/ws/cam")
async def websocket_cam_endpoint(ws: WebSocket):
    token = (cfg.cam_ws_token or "").strip()
    if token:
        if ws.query_params.get("token") != token:
            await ws.close(code=4401)
            return
    await ws.accept()
    await cam_hub.register(ws)
    try:
        while True:
            await ws.receive()
    except WebSocketDisconnect:
        pass
    finally:
        await cam_hub.unregister(ws)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8765, reload=False)
