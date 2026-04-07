"""
Demo: đọc MJPEG từ ESP32-CAM + YOLO (OpenCV imshow).
Cùng ý tưởng đọc luồng với dashboard/stream_reader.py.

Chạy (từ gốc repo, đã cài ultralytics + opencv-python):
  pip install ultralytics opencv-python numpy
  python scripts/yolo_mjpeg_viewer.py

Chỉnh stream_url và đặt model tại models/yolo_best.pt (hoặc sửa DEFAULT_MODEL bên dưới).
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import cv2
import numpy as np
import urllib.request
from ultralytics import YOLO

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = REPO_ROOT / "models" / "yolo_best.pt"


class BulletproofStream:
    def __init__(self, url: str):
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def _update(self):
        while self.running:
            try:
                stream = urllib.request.urlopen(self.url, timeout=3)
                bytes_data = b""

                while self.running:
                    chunk = stream.read(16384)
                    if not chunk:
                        break

                    bytes_data += chunk

                    a = bytes_data.find(b"\xff\xd8")
                    if a != -1:
                        b = bytes_data.find(b"\xff\xd9", a)

                        if b != -1:
                            jpg = bytes_data[a : b + 2]
                            bytes_data = bytes_data[b + 2 :]

                            img = cv2.imdecode(
                                np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                            )
                            if img is not None:
                                with self.lock:
                                    self.frame = img

                    if len(bytes_data) > 600000:
                        bytes_data = b""

            except Exception as e:
                print(f"\n[MẠNG] Chập chờn, đang nối lại... ({e})")
                time.sleep(1)

    def read(self):
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
            return None

    def stop(self):
        self.running = False


def main():
    stream_url = "http://192.168.137.203:81/stream"
    model_path = str(DEFAULT_MODEL)

    print("=========================================")
    print("    HỆ THỐNG NHẬN DIỆN LỬA - CHÂN ÁI     ")
    print("=========================================")

    if not Path(model_path).is_file():
        print(f"Lỗi: không thấy model {model_path}")
        return

    print("Đang tải não bộ AI...")
    model = YOLO(model_path)

    print(f"Đang kết nối tới Camera: {stream_url}...")

    cam_stream = BulletproofStream(stream_url).start()

    print("Chờ 2 giây để tải khung hình đầu tiên...")
    time.sleep(2)

    if cam_stream.read() is None:
        print("Lỗi: Vẫn không thấy ảnh. Kiểm tra lại IP hoặc cắm lại điện ESP32-CAM!")
        cam_stream.stop()
        return

    print("✅ KẾT NỐI THÀNH CÔNG! Đã lên hình. (Bấm 'q' để thoát)")

    try:
        while True:
            frame = cam_stream.read()

            if frame is not None:
                results = model.predict(
                    source=frame, imgsz=416, conf=0.5, verbose=False
                )

                annotated_frame = results[0].plot()

                cv2.imshow("He Thong Nhan Dien Lua", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cam_stream.stop()
        cv2.destroyAllWindows()
        print("Đã đóng hệ thống.")


if __name__ == "__main__":
    main()
