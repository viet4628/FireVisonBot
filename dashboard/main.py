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


class RuntimeConfig:
    stream_url: str = "http://192.168.137.203:81/stream"
    # IP ESP32-S3 (đổi khi DHCP cấp IP khác). Không dùng .1 (gateway laptop).
    robot_status_url: str = "http://192.168.137.60:8080/api/status"
    yolo_imgsz: int = 416
    yolo_conf: float = 0.45
    infer_interval_s: float = 0.12
    robot_poll_s: float = 0.4
    # Timeout HTTP tới ESP32-S3 (s). Quá ngắn dễ thấy "timeout" dù IP đúng.
    robot_http_timeout_s: float = 12.0
    ai_feed_jpeg_quality: int = 95  # /ai_feed MJPEG (60–98, cao = nét hơn trên dashboard)
    # Đẩy độ tin cậy YOLO (0…1) lên ESP32-S3 để bật relay khi kết hợp IR trái + >= 65%.
    robot_ai_push_url: str = "http://192.168.137.60:8080/api/ai_fire"


cfg = RuntimeConfig()


def _esp_http_timeout() -> httpx.Timeout:
    t = max(3.0, float(cfg.robot_http_timeout_s))
    return httpx.Timeout(t)


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
                if main_loop and main_loop.is_running():
                    asyncio.run_coroutine_threadsafe(
                        hub.broadcast({"type": "sensors", "data": data}),
                        main_loop,
                    )
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
    restart_stream_reader()

    yield

    worker_stop.set()
    global stream_reader
    with stream_lock:
        if stream_reader is not None:
            stream_reader.stop()
            stream_reader = None


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
        "stream_url": cfg.stream_url,
        "robot_status_url": cfg.robot_status_url,
        "yolo_imgsz": cfg.yolo_imgsz,
        "yolo_conf": cfg.yolo_conf,
        "infer_interval_s": cfg.infer_interval_s,
        "robot_poll_s": cfg.robot_poll_s,
        "ai_feed_jpeg_quality": cfg.ai_feed_jpeg_quality,
        "robot_ai_push_url": cfg.robot_ai_push_url,
        "robot_http_timeout_s": cfg.robot_http_timeout_s,
        "model_path": str(DEFAULT_MODEL),
        "model_exists": DEFAULT_MODEL.is_file(),
    }


@app.post("/api/config")
async def api_config_post(request: Request):
    body = await request.json()
    if "stream_url" in body:
        cfg.stream_url = str(body["stream_url"])
    if "robot_status_url" in body:
        cfg.robot_status_url = str(body["robot_status_url"])
    if "yolo_imgsz" in body:
        cfg.yolo_imgsz = int(body["yolo_imgsz"])
    if "yolo_conf" in body:
        cfg.yolo_conf = float(body["yolo_conf"])
    if "infer_interval_s" in body:
        cfg.infer_interval_s = float(body["infer_interval_s"])
    if "robot_poll_s" in body:
        cfg.robot_poll_s = float(body["robot_poll_s"])
    if "ai_feed_jpeg_quality" in body:
        q = int(body["ai_feed_jpeg_quality"])
        cfg.ai_feed_jpeg_quality = max(60, min(98, q))
    if "robot_ai_push_url" in body:
        cfg.robot_ai_push_url = str(body["robot_ai_push_url"])
    if "robot_http_timeout_s" in body:
        cfg.robot_http_timeout_s = max(3.0, float(body["robot_http_timeout_s"]))
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


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8765, reload=False)
