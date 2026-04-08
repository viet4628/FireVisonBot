#pragma once

#include <stdint.h>

// ===== INIT =====
void motor_init(void);

// ===== CONTROL =====
void motor_forward(uint32_t duty);
void motor_backward(uint32_t duty);
void motor_turn_left(uint32_t duty);
void motor_turn_right(uint32_t duty);
void motor_stop(void);

/**
 * Tiến cầm (differential drive): cả hai bánh đều chạy tiến nhưng duty khác nhau.
 * left_duty  = xung bánh trái (0 … DUTY_MAX)
 * right_duty = xung bánh phải (0 … DUTY_MAX)
 */
void motor_drive_curve(uint32_t left_duty, uint32_t right_duty);