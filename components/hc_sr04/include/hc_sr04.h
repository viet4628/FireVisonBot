#pragma once

#include <stdint.h>
#include <stdbool.h>

void hc_sr04_init(void);
void hc_sr04_start_monitoring(uint32_t period_ms);
float hc_sr04_get_last_distance_cm(void);
bool hc_sr04_is_target_near(float threshold_cm);
