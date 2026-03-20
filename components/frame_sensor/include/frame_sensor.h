#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FLAME_SENSOR_LEFT = 0,
    FLAME_SENSOR_RIGHT,
    FLAME_SENSOR_COUNT
} flame_sensor_id_t;

void frame_sensor_init(void);
bool frame_sensor_is_fire_detected(flame_sensor_id_t id);
bool frame_sensor_any_fire_detected(void);
void frame_sensor_start_monitoring(uint32_t period_ms);
