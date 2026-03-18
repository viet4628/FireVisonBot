#pragma once

#include <stdint.h>

// ===== INIT =====
void motor_init(void);

// ===== CONTROL =====
void motor_forward(uint32_t duty);
void motor_backward(uint32_t duty);
void motor_stop(void);