# FireVisionBot (ESP-IDF / ESP32-S3)

Ngôn ngữ:
- Tiếng Việt: `README.vi.md`
- English: [`README.md`](README.md)

Repository này chứa firmware ESP-IDF (`fire_robot`) cho xe robot chữa cháy tự động dùng ESP32-S3 N16R8.

## Mô Tả Hệ Thống

### Phần cứng
- **ESP32-S3 N16R8** – MCU trung tâm xử lý chính
- **ESP32-CAM (AI Thinker)** – quay hình ảnh, stream cho ESP32-S3 để xác minh AI
- **L298N** – điều khiển 4 động cơ DC (2 kênh, mỗi kênh 2 motor song song)
- **4 Servo**:
  - 2 servo quét cảm biến lửa IR (quay 360° ngược hướng nhau)
  - 2 servo FPV (pan/tilt) điều khiển góc quay ESP32-CAM
- **2 Cảm biến lửa IR** – gắn trên 2 servo quét
- **1 HC-SR04** – cảm biến siêu âm kiểm tra khoảng cách / vật cản
- **1 Relay** – điều khiển động cơ bơm nước
- **1 Còi buzzer** – báo động khi phát hiện lửa
- **Model AI (FOMO MobileNetV2 0.35)** – nhận diện lửa, đã train cho ESP32-S3

### Cơ chế hoạt động

```text
IDLE_SCAN → FIRE_DETECTED → ORIENT_CAMERA → AI_VERIFY → APPROACH → EXTINGUISH
                                                            ↑           │
                                                            └───────────┘
                                              (mất lửa) → IDLE_SCAN
```

1. **Chế độ ngủ (IDLE_SCAN)**: 2 servo quét IR quay qua lại, quét lửa liên tục
2. **Phát hiện lửa (FIRE_DETECTED)**: Cảm biến IR phát hiện → khóa servo → xác nhận 300ms
3. **Xoay camera (ORIENT_CAMERA)**: 2 servo FPV xoay camera hướng về phía lửa
4. **Xác minh AI (AI_VERIFY)**: *Placeholder* – ESP32-CAM chụp hình → chạy model AI xác nhận
5. **Tiếp cận (APPROACH)**: Xe di chuyển về phía lửa, liên tục bám hướng bằng IR
6. **Dập lửa (EXTINGUISH)**: Đến gần đủ → dừng xe → bật bơm nước → theo dõi lửa tắt

## Sơ Đồ Chân (GPIO)

### Motor – L298N (`components/motor/motor.c`)

| Chức năng | GPIO | Ghi chú |
| --- | --- | --- |
| MOTOR1 PWM (LEFT) | 9 | Tốc độ bên trái |
| MOTOR1 RPWM | 10 | Chiều thuận bên trái |
| MOTOR1 LPWM | 11 | Chiều nghịch bên trái |
| MOTOR2 PWM (RIGHT) | 14 | Tốc độ bên phải |
| MOTOR2 RPWM | 12 | Chiều thuận bên phải |
| MOTOR2 LPWM | 13 | Chiều nghịch bên phải |

### Servo (`components/servo/servo.c`)

| Chức năng | GPIO | Ghi chú |
| --- | --- | --- |
| Scan Left (IR trái) | 15 | MCPWM Group 0 |
| Scan Right (IR phải) | 21 | MCPWM Group 1 |
| FPV Pan (Servo Dưới - Xoay Ngang) | 38 | MCPWM Group 0 |
| FPV Tilt (Servo Trên - Xoay Dọc) | 39 | MCPWM Group 1 |

### Cảm Biến Lửa IR (`components/frame_sensor/frame_sensor.c`)

| Chức năng | GPIO |
| --- | --- |
| Flame Sensor LEFT | 4 |
| Flame Sensor RIGHT | 6 |

### HC-SR04 Siêu Âm (`components/hc_sr04/hc_sr04.c`)

| Chức năng | GPIO |
| --- | --- |
| TRIG | 5 |
| ECHO | 18 |

### Relay – Bơm Nước (`components/relay/relay.c`)

| Chức năng | GPIO |
| --- | --- |
| Relay (Active HIGH) | 17 |

### Buzzer (`main/app_main.c`)

| Chức năng | GPIO |
| --- | --- |
| Buzzer (Active LOW) | 16 |

## Yêu Cầu

- ESP-IDF `v5.x` (đã build với `v5.5.2`)
- Board ESP32-S3 N16R8
- Driver motor L298N
- 4× Servo SG90/MG90S (50 Hz, xung 450-2400 µs)
- 2× Cảm biến lửa IR (digital output, active LOW)
- 1× HC-SR04
- 1× Relay module
- 1× Buzzer (active LOW)

## Build và Flash

```bash
cd /path/to/FireVisonBot
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Thay `/dev/ttyUSB0` bằng cổng serial của bạn (Windows: `COM3`, `COM4`, ...).

## Cấu Trúc Dự Án

```text
FireVisonBot/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── app_main.c              ← State machine chính
├── components/
│   ├── motor/
│   │   ├── CMakeLists.txt
│   │   ├── include/motor.h
│   │   └── motor.c             ← L298N driver (forward/backward/turn)
│   ├── servo/
│   │   ├── CMakeLists.txt
│   │   ├── include/servo.h
│   │   └── servo.c             ← 4 servo MCPWM driver
│   ├── frame_sensor/
│   │   ├── CMakeLists.txt
│   │   ├── include/frame_sensor.h
│   │   └── frame_sensor.c      ← IR flame sensor driver
│   ├── hc_sr04/
│   │   ├── CMakeLists.txt
│   │   ├── include/hc_sr04.h
│   │   └── hc_sr04.c           ← Ultrasonic sensor driver
│   └── relay/
│       ├── CMakeLists.txt
│       ├── include/relay.h
│       └── relay.c              ← Water pump relay driver
├── trained_model/
│   ├── fire_model_int8.tflite   ← AI model cho ESP32-S3
│   └── fire_model_data.h        ← Model as C header
└── sdkconfig
```

## Điểm Có Thể Tùy Chỉnh

- Logic chuyển động và timing: [`main/app_main.c`](main/app_main.c)
- Chân motor, tần số/độ phân giải PWM: [`components/motor/motor.c`](components/motor/motor.c)
- Chân servo, dải xung: [`components/servo/servo.c`](components/servo/servo.c)
- Chân cảm biến lửa: [`components/frame_sensor/frame_sensor.c`](components/frame_sensor/frame_sensor.c)
- Chân siêu âm: [`components/hc_sr04/hc_sr04.c`](components/hc_sr04/hc_sr04.c)
- Chân relay: [`components/relay/relay.c`](components/relay/relay.c)

## TODO

- [ ] Kết nối ESP32-CAM (UART/WiFi) để nhận stream hình ảnh
- [ ] Tích hợp TFLite model (`fire_model_int8.tflite`) vào state AI_VERIFY
- [ ] Thêm điều khiển từ xa (WiFi/Bluetooth)
- [ ] Thêm chế độ điều khiển manual

## Checklist An Toàn

- Đảm bảo ESP32, L298N, servo và relay dùng chung GND
- Không cấp nguồn động cơ trực tiếp từ board ESP32
- Dùng nguồn riêng cho motor (7-12V) và servo (5V)
- Relay nên có diode chống ngược (flyback diode)
- Kiểm tra GPIO pin map trước khi flash lần đầu
