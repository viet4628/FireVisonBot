# FireVisionBot (ESP-IDF / ESP32-S3)

Language:
- English: `README.md`
- Tiếng Việt: [`README.vi.md`](README.vi.md)

This repository contains a small ESP-IDF firmware project (`fire_robot`) for an ESP32-S3 based mobile robot:

- 2 DC motors controlled by PWM + direction pins
- 1 servo controlled by MCPWM
- a simple repeating movement pattern in `app_main`

## Current Behavior

Main loop in [`main/app_main.c`](main/app_main.c):

1. Move forward at duty `800`, set servo to `30` degrees, wait `3s`
2. Stop motors, wait `1s`
3. Move backward at duty `800`, set servo to `150` degrees, wait `3s`
4. Repeat forever

## Hardware Pin Mapping (from source)

### Motor component (`components/motor/motor.c`)

| Function | GPIO |
| --- | --- |
| MOTOR1 PWM | 9 |
| MOTOR1 RPWM | 10 |
| MOTOR1 LPWM | 11 |
| MOTOR2 PWM | 12 |
| MOTOR2 RPWM | 13 |
| MOTOR2 LPWM | 14 |

### Servo component (`components/servo/servo.c`)

| Function | GPIO |
| --- | --- |
| SERVO SIGNAL | 14 |

## Important Note

There is currently a GPIO conflict in code: both `MOTOR2 LPWM` and `SERVO SIGNAL` use GPIO `14`.

Before running on real hardware, update one of these pin assignments to avoid signal conflicts.

## Requirements

- ESP-IDF `v5.x` (project was built locally with `v5.5.2`)
- ESP32-S3 board
- Motor driver wiring compatible with separate PWM + direction control
- 50 Hz hobby servo (default pulse range `600-2400 us`)

## Build and Flash

```bash
cd /path/to/FireVisonBot
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port.

## Project Structure

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

## Where to Customize

- Movement logic and timing: [`main/app_main.c`](main/app_main.c)
- Motor pins, PWM frequency/resolution: [`components/motor/motor.c`](components/motor/motor.c)
- Servo pin, pulse range, sweep behavior: [`components/servo/servo.c`](components/servo/servo.c)

## Quick Safety Checklist

- Verify shared ground between ESP32, motor driver, and servo power supply
- Do not power motors directly from the ESP32 board
- Confirm pin map before first flash (especially GPIO14 conflict)
