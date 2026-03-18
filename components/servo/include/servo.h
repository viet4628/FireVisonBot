#pragma once

#include <stdint.h>

// ===== INIT =====
void servo_init(void);

// ===== CONTROL =====
void servo_set_angle(float angle);

// Optional
void servo_sweep_start(void);