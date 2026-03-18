# FireVisionBot (ESP-IDF / ESP32-S3)

Ngôn ngữ:
- Tiếng Việt: `README.vi.md`
- English: [`README.md`](README.md)

Repository này chứa firmware ESP-IDF (`fire_robot`) cho robot di động dùng ESP32-S3:

- 2 động cơ DC điều khiển bằng PWM + chân định hướng
- 1 servo điều khiển bằng MCPWM
- vòng lặp chuyển động lặp lại trong `app_main`

## Hành Vi Hiện Tại

Vòng lặp chính trong [`main/app_main.c`](main/app_main.c):

1. Chạy tiến với duty `800`, đặt servo `30` độ, chờ `3s`
2. Dừng động cơ, chờ `1s`
3. Chạy lùi với duty `800`, đặt servo `150` độ, chờ `3s`
4. Lặp vô hạn

## Sơ Đồ Chân (theo source)

### Motor (`components/motor/motor.c`)

| Chức năng | GPIO |
| --- | --- |
| MOTOR1 PWM | 9 |
| MOTOR1 RPWM | 10 |
| MOTOR1 LPWM | 11 |
| MOTOR2 PWM | 12 |
| MOTOR2 RPWM | 13 |
| MOTOR2 LPWM | 14 |

### Servo (`components/servo/servo.c`)

| Chức năng | GPIO |
| --- | --- |
| Tín hiệu SERVO | 14 |

## Lưu Ý Quan Trọng

Hiện đang có xung đột GPIO trong code: `MOTOR2 LPWM` và `SERVO SIGNAL` đều dùng GPIO `14`.

Trước khi chạy trên phần cứng thật, cần đổi một trong hai chân này để tránh xung đột tín hiệu.

## Yêu Cầu

- ESP-IDF `v5.x` (đã build cục bộ với `v5.5.2`)
- Board ESP32-S3
- Driver động cơ tương thích kiểu điều khiển PWM + direction
- Servo hobby 50 Hz (xung mặc định `600-2400 us`)

## Build và Flash

```bash
cd /path/to/FireVisonBot
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Thay `/dev/ttyUSB0` bằng cổng serial của bạn.

## Cấu Trúc Dự Án

```text
FireVisonBot/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── app_main.c
├── components/
│   ├── motor/
│   │   ├── CMakeLists.txt
│   │   ├── include/motor.h
│   │   └── motor.c
│   └── servo/
│       ├── CMakeLists.txt
│       ├── include/servo.h
│       └── servo.c
└── sdkconfig
```

## Điểm Có Thể Tùy Chỉnh

- Logic chuyển động và timing: [`main/app_main.c`](main/app_main.c)
- Chân motor, tần số/độ phân giải PWM: [`components/motor/motor.c`](components/motor/motor.c)
- Chân servo, dải xung, sweep behavior: [`components/servo/servo.c`](components/servo/servo.c)

## Checklist An Toàn Nhanh

- Đảm bảo ESP32, driver motor và nguồn servo dùng chung GND
- Không cấp nguồn động cơ trực tiếp từ board ESP32
- Kiểm tra pin map trước khi flash lần đầu (đặc biệt xung đột GPIO14)
