#pragma once

#include <stdint.h>

typedef enum {
    SERVO_SCAN_LEFT = 0,   // IR flame sensor – left side
    SERVO_SCAN_RIGHT,      // IR flame sensor – right side
    SERVO_FPV_PAN,         // FPV camera – horizontal (pan)
    SERVO_FPV_TILT,        // FPV camera – vertical   (tilt)
    SERVO_COUNT
} servo_id_t;

void servo_init(void);

void servo_set_angle(servo_id_t id, float angle);
float servo_get_angle(servo_id_t id);

// Keep old API for backward compatibility (maps to SERVO_SCAN_LEFT).
void servo_set_primary_angle(float angle);

// Optional sweep test for two scan servos.
void servo_sweep_start(void);