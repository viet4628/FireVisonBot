# 🔥 FireVisionBot (ESP32-S3 AI Firefighting Robot)

[![Language: Vietnamese](https://img.shields.io/badge/Language-Vietnamese-red.svg)](README.vi.md)
[![Language: English](https://img.shields.io/badge/Language-English-blue.svg)](README.md)
[![Framework: ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF%20v5.x-orange.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/index.html)

Dự án robot chữa cháy tự động sử dụng **ESP32-S3** kết hợp thị giác máy tính (**YOLOv8**) để nhận diện và dập lửa chính xác. Hệ thống sử dụng cảm biến lửa IR để quét radar và xác thực lại bằng AI qua luồng video từ **ESP32-CAM**.

---

## 🏗️ Kiến Trúc Hệ Thống

### 1. Phần Cứng (Hardware)
- **MCU Chính**: ESP32-S3 N16R8 (Xử lý logic, WiFi, Telemetry).
- **Camera**: ESP32-CAM (Stream MJPEG VGA ổn định qua WiFi).
- **Di chuyển**: 4 động cơ DC + Driver L298N (Điều khiển PWM độc lập).
- **Cảm biến quét**: 
  - 2 Servo quét radar gắn 2 cảm biến lửa IR (Tăng góc quét 180°).
  - 2 Servo FPV (Pan/Tilt) điều khiển hướng nhìn của Camera.
- **An toàn & Dập lửa**:
  - Cảm biến siêu âm HC-SR04 (Tránh vật cản).
  - Relay + Bơm nước (Dập lửa).
  - Còi Buzzer (Báo động).

### 2. Sơ Đồ Chân (GPIO Mapping) - `board_hw.h`

| Linh kiện | Chân GPIO | Cấu hình / Ghi chú |
| --- | --- | --- |
| **Motor Left** | 9 (PWM), 10 (R), 11 (L) | LEDC Timer 1, Ch 4 |
| **Motor Right** | 14 (PWM), 12 (R), 13 (L) | LEDC Timer 1, Ch 5 |
| **Servo Scan L/R** | 15, 7 | LEDC Timer 0, Ch 0-1 |
| **Servo FPV Pan/Tilt**| 38, 39 | LEDC Timer 0, Ch 2-3 |
| **Flame IR L/R** | 4, 6 | Active Low (Pull-up) |
| **HC-SR04** | 5 (Trig), 18 (Echo) | MCPWM Capture |
| **Relay Bơm** | 17 | Active High |
| **Buzzer** | 16 | Active High |

---

## 🤖 Nguyên Lý Hoạt Động (State Machine)

Robot vận hành dựa trên 3 trạng thái chính trong [`app_main.c`](main/app_main.c):

1.  **STATE_PATROL (Tuần tra)**:
    - 2 Servo quét IR di chuyển ngược hướng nhau từ -15° đến 180°.
    - Nếu `PATROL_ENABLE_DRIVE` bật, xe tự tiến và tránh vật cản bằng HC-SR04.
    - Chờ tín hiệu từ IR hoặc Camera AI.

2.  **STATE_CAMERA_APPROACH (Tiếp cận bằng AI)**:
    - Kích hoạt khi Camera thấy lửa nhưng IR chưa bắt được.
    - Sử dụng thuật toán **Differential Drive**: Xe tiến và lái mượt mà về phía lửa dựa trên vị trí `x_ratio` từ YOLOv8.
    - Servo quét IR tăng tốc độ (x1.8) để nhanh chóng khóa mục tiêu.

3.  **STATE_EXTINGUISH (Dập lửa)**:
    - Khóa chặt góc Servo tại vị trí phát hiện lửa.
    - Còi báo động kêu liên tục.
    - **Điều kiện bật bơm**: (IR phát hiện lửa) **VÀ** (AI xác nhận Confidence > 65%).

---

## 💻 Dashboard & AI (FastAPI + YOLOv8)

Backend chạy trên Laptop để gánh tải xử lý AI nặng:
- **Ngôn ngữ**: Python 3.10+ (FastAPI).
- **AI Model**: YOLOv8 (`models/yolo_best.pt`) nhận diện lửa realtime.
- **Tính năng**:
    - Nhận luồng MJPEG từ ESP32-CAM.
    - Vẽ BoundBox và gửi kết quả xác thực về ESP32-S3 qua HTTP POST.
    - Dashboard Web xem video feed, telemetry và điều khiển thông số.

### Chạy Dashboard:
```bash
cd dashboard
python -m venv .venv
# Activate venv (Windows: .\.venv\Scripts\activate | Linux: source .venv/bin/activate)
pip install -r requirements.txt
python main.py
```

---

## 🛠️ Hướng Dẫn Cài Đặt Firmware

### Yêu cầu:
- ESP-IDF v5.x (Khuyến nghị v5.2+).
- Cài đặt môi trường `idf.py`.

### Build & Flash:
```bash
# Đặt target
idf.py set-target esp32s3
# Build dự án
idf.py build
# Flash và xem log
idf.py -p COMx flash monitor
```
*(Thay `COMx` bằng cổng thực tế trên máy bạn)*

---

## ⚙️ Công Thức Tính Toán Quan Trọng

### 1. Khoảng cách Siêu âm:
`Khoảng cách (cm) = Thời gian (µs) × 0.0343 / 2`
*(Sử dụng MCPWM capture trong firmware để có độ chính xác cao)*

### 2. Góc Servo SG90:
`Pulse Width (µs) = ((Góc / 180) × (2500 - 500)) + 500`
- **0°**: ~500µs (Xung mức cao).
- **90°**: ~1500µs.
- **180°**: ~2500µs.

---

## 📝 Ghi chú Kỹ thuật
- **Nguồn điện**: Tuyệt đối không dùng chung nguồn 3.3V của ESP cho Motor/Servo. Sử dụng nguồn riêng 7.4V (Lipo 2S) cho Motor và 5V ổn định cho Servo/ESP.
- **GND**: Phải nối chung GND của tất cả các module.
- **Chế độ FPV**: Servo Pan (dưới) sẽ bám theo góc của cảm biến lửa IR trái để Camera luôn hướng về tiêu điểm dập lửa.
