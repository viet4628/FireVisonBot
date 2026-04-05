"""Đọc MJPEG từ ESP32-CAM — urllib + tách JPEG SOI/EOI + giới hạn buffer (cùng ý tưởng scripts/yolo_mjpeg_viewer.py)."""

from __future__ import annotations

import threading
import time
import urllib.request

import cv2
import numpy as np


class MjpegStreamReader:
    """Luồng nền: urlopen → read chunk → tìm SOI/EOI JPEG → imdecode → frame mới nhất."""

    def __init__(self, url: str):
        self.url = url
        self._frame = None
        self._running = True
        self._lock = threading.Lock()

    def start(self) -> MjpegStreamReader:
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def stop(self) -> None:
        self._running = False

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
                print(f"[stream] Chập chờn, đang nối lại... ({exc})")
            finally:
                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass
            if not self._running:
                break
            time.sleep(1.0 if had_error else 0.15)
