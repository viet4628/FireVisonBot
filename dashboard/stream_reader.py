"""Đọc MJPEG từ ESP32-CAM — urllib + tách JPEG SOI/EOI + giới hạn buffer (cùng ý tưởng scripts/yolo_mjpeg_viewer.py)."""

from __future__ import annotations

import threading
import time
import urllib.request
import urllib.error

import cv2
import numpy as np


class MjpegStreamReader:
    """Luồng nền: urlopen → read chunk → tìm SOI/EOI JPEG → imdecode → frame mới nhất."""

    def __init__(self, url: str):
        self.url = url
        self._frame = None
        self._running = True
        self._lock = threading.Lock()
        self._stream = None
        self._err_ts = 0.0

    def start(self) -> MjpegStreamReader:
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def stop(self) -> None:
        self._running = False
        s = self._stream
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def read(self):
        with self._lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def _loop(self):
        while self._running:
            stream = None
            had_error = False
            try:
                stream = urllib.request.urlopen(self.url, timeout=3)
                self._stream = stream
                buf = b""
                # Chunk lớn + buffer lớn: JPEG VGA/SVGA chất lượng cao >150KB;
                # cắt buffer quá sớm làm mất EOI → imdecode vỡ khối / mosaic.
                _chunk = 16384
                _max_buf = 600000
                while self._running:
                    chunk = stream.read(_chunk)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(b"\xff\xd8")
                    if a != -1:
                        b = buf.find(b"\xff\xd9", a)
                        if b != -1:
                            jpg = buf[a : b + 2]
                            buf = buf[b + 2 :]
                            img = cv2.imdecode(
                                np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                            )
                            if img is not None:
                                with self._lock:
                                    self._frame = img
                    if len(buf) > _max_buf:
                        buf = b""
            except Exception as exc:
                had_error = True
                # Nếu camera đang tắt: /stream trả 503 (stream_disabled) — không spam log.
                quiet = False
                if isinstance(exc, urllib.error.HTTPError) and exc.code == 503:
                    quiet = True
                now = time.monotonic()
                if not quiet and (now - self._err_ts) >= 2.0:
                    self._err_ts = now
                    print(f"[stream] Chập chờn, đang nối lại... ({exc})")
            finally:
                self._stream = None
                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass
            if not self._running:
                break
            # Nếu lỗi 503 do camera tắt, nghỉ lâu hơn để đỡ tốn CPU.
            time.sleep(1.4 if had_error else 0.15)
