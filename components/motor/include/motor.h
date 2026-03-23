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