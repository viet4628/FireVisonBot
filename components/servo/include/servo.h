#pragma once

#include <stdint.h>

typedef enum {
    SERVO_SCAN_LEFT = 0,
    SERVO_SCAN_RIGHT,
    SERVO_WATER_NOZZLE,
    SERVO_COUNT
} servo_id_t;

void servo_init(void);

void servo_set_angle(servo_id_t id, float angle);
float servo_get_angle(servo_id_t id);

// Keep old API for backward compatibility (maps to SERVO_SCAN_LEFT).
void servo_set_primary_angle(float angle);

// Optional sweep test for two scan servos.
void servo_sweep_start(void);