/**
 * @file board_hw.h
 * Bản đồ phần cứng + phân bổ LEDC cho FireVisonBot (một nơi chỉnh để đồng bộ toàn hệ thống).
 *
 * Quy tắc LEDC:
 * - Servo: LEDC_TIMER_0, kênh 0–3, 50 Hz, 14 bit — không đổi timer này cho motor.
 * - Motor: LEDC_TIMER_1, kênh 4–5 — tần số PWM độc lập với servo.
 */
#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

/* --- Cảm biến lửa IR (mức thấp = có lửa, pull-up) --- */
#define BOARD_GPIO_FLAME_LEFT   GPIO_NUM_4
#define BOARD_GPIO_FLAME_RIGHT  GPIO_NUM_6
/**
 * Một số module IR flame xuất mức LOW khi có lửa (active-low),
 * một số khác xuất mức HIGH khi có lửa (active-high).
 * Đặt 1 nếu active-low, đặt 0 nếu active-high.
 */
#define BOARD_FLAME_ACTIVE_LOW  1

/* --- HC-SR04 (MCPWM capture) --- */
#define BOARD_GPIO_SR04_TRIG    GPIO_NUM_5
#define BOARD_GPIO_SR04_ECHO    GPIO_NUM_18

/* --- Relay bơm --- */
#define BOARD_GPIO_RELAY        GPIO_NUM_17
#define BOARD_RELAY_ON_LEVEL    1

/* --- Buzzer --- */
#define BOARD_GPIO_BUZZER       GPIO_NUM_16
#define BOARD_BUZZER_ON_LEVEL   1

/* --- DC motor (L298N kiểu: ENA/ENB = PWM, IN1/IN2 = hướng) --- */
#define BOARD_GPIO_MOTOR_PWM_LEFT    GPIO_NUM_9
#define BOARD_GPIO_MOTOR_RPWM_LEFT   GPIO_NUM_10
#define BOARD_GPIO_MOTOR_LPWM_LEFT   GPIO_NUM_11
#define BOARD_GPIO_MOTOR_PWM_RIGHT   GPIO_NUM_14
#define BOARD_GPIO_MOTOR_RPWM_RIGHT  GPIO_NUM_12
#define BOARD_GPIO_MOTOR_LPWM_RIGHT  GPIO_NUM_13

#define BOARD_MOTOR_LEDC_TIMER        LEDC_TIMER_1
#define BOARD_MOTOR_LEDC_CH_LEFT      LEDC_CHANNEL_4
#define BOARD_MOTOR_LEDC_CH_RIGHT     LEDC_CHANNEL_5
/**
 * ~6.5 kHz: lực quay tốt hơn 10–20 kHz trên L298N+DC (đổi lại dễ nghe rít hơn).
 */
#define BOARD_MOTOR_LEDC_FREQ_HZ      6500
#define BOARD_MOTOR_LEDC_TIMER_BITS   LEDC_TIMER_10_BIT
#define BOARD_MOTOR_DUTY_MAX          1023U

/* --- Servo (SG90-class) --- */
#define BOARD_GPIO_SERVO_SCAN_LEFT   GPIO_NUM_15
#define BOARD_GPIO_SERVO_SCAN_RIGHT  GPIO_NUM_7
#define BOARD_GPIO_SERVO_FPV_PAN     GPIO_NUM_38
#define BOARD_GPIO_SERVO_FPV_TILT    GPIO_NUM_39

#define BOARD_SERVO_LEDC_TIMER         LEDC_TIMER_0
#define BOARD_SERVO_LEDC_FREQ_HZ       50
#define BOARD_SERVO_LEDC_TIMER_BITS    LEDC_TIMER_14_BIT
#define BOARD_SERVO_DUTY_MAX_FLOAT     16383.0f

#define BOARD_SERVO_CH_SCAN_LEFT   LEDC_CHANNEL_0
#define BOARD_SERVO_CH_SCAN_RIGHT  LEDC_CHANNEL_1
#define BOARD_SERVO_CH_FPV_PAN     LEDC_CHANNEL_2
#define BOARD_SERVO_CH_FPV_TILT    LEDC_CHANNEL_3

void board_hw_log_layout(void);
