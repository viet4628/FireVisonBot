#pragma once

#include <stdbool.h>
#include <stdint.h>

void frame_sensor_init(void);
bool frame_sensor_is_fire_detected(void);
void frame_sensor_start_monitoring(uint32_t period_ms);
