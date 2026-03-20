#include "motor.h"
#include "servo.h"
#include "frame_sensor.h"
#include "hc_sr04.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FIRE_CTRL";
#define BUZZER_GPIO GPIO_NUM_16
// If your buzzer is active-low, set BUZZER_ON_LEVEL to 0.
#define BUZZER_ON_LEVEL 0

static void buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_GPIO, !BUZZER_ON_LEVEL);
}

static void buzzer_update(bool fire_confirmed)
{
    static TickType_t next_toggle_ticks = 0;
    static bool buzzer_on = false;
    const TickType_t on_ticks = pdMS_TO_TICKS(120);
    const TickType_t off_ticks = pdMS_TO_TICKS(220);
    TickType_t now = xTaskGetTickCount();

    if (!fire_confirmed) {
        buzzer_on = false;
        next_toggle_ticks = 0;
        gpio_set_level(BUZZER_GPIO, !BUZZER_ON_LEVEL);
        return;
    }

    if (next_toggle_ticks == 0 || now >= next_toggle_ticks) {
        buzzer_on = !buzzer_on;
        gpio_set_level(BUZZER_GPIO, buzzer_on ? BUZZER_ON_LEVEL : !BUZZER_ON_LEVEL);
        next_toggle_ticks = now + (buzzer_on ? on_ticks : off_ticks);
    }
}

static float clamp_angle(float angle, float min_angle, float max_angle)
{
    if (angle < min_angle) {
        return min_angle;
    }
    if (angle > max_angle) {
        return max_angle;
    }
    return angle;
}

static void fire_tracking_task(void *arg)
{
    (void)arg;
    const float min_angle = -15.0f;
    const float max_angle = 180.0f;
    const float scan_step = 2.0f;
    const float track_step = 0.9f;
    const float stop_distance_cm = 25.0f;
    const TickType_t lock_delay_ticks = pdMS_TO_TICKS(300);
    const TickType_t unlock_delay_ticks = pdMS_TO_TICKS(3000);
    const TickType_t track_log_period_ticks = pdMS_TO_TICKS(50);
    float scan_angle = min_angle;
    float target_angle = 90.0f;
    int direction = 1;
    bool locked = false;
    bool fire_confirmed = false;
    TickType_t last_track_log_ticks = 0;
    uint8_t left_streak = 0;
    uint8_t right_streak = 0;
    const uint8_t confirm_streak = 2;

    while (1) {
        if (!locked) {
            fire_confirmed = false;
            buzzer_update(false);

            servo_set_angle(SERVO_SCAN_LEFT, scan_angle);
            servo_set_angle(SERVO_SCAN_RIGHT, max_angle - (scan_angle - min_angle));

            if (frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT)) {
                float candidate_angle = servo_get_angle(SERVO_SCAN_LEFT);
                ESP_LOGI(TAG, "Fire detected on LEFT, wait 0.3s before lock");
                vTaskDelay(lock_delay_ticks);
                if (frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT)) {
                    target_angle = candidate_angle;
                    locked = true;
                    fire_confirmed = true;
                    ESP_LOGI(TAG, "Target locked from LEFT sensor at %.1f deg", target_angle);
                }
            } else if (frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT)) {
                float candidate_angle = servo_get_angle(SERVO_SCAN_RIGHT);
                ESP_LOGI(TAG, "Fire detected on RIGHT, wait 0.3s before lock");
                vTaskDelay(lock_delay_ticks);
                if (frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT)) {
                    target_angle = candidate_angle;
                    locked = true;
                    fire_confirmed = true;
                    ESP_LOGI(TAG, "Target locked from RIGHT sensor at %.1f deg", target_angle);
                }
            } else {
                scan_angle += direction * scan_step;
                if (scan_angle >= max_angle || scan_angle <= min_angle) {
                    direction *= -1;
                }
            }
        } else {
            bool left = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
            bool right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
            fire_confirmed = left || right;
            buzzer_update(fire_confirmed);

            // Dynamic re-lock: keep adjusting target angle while flame moves.
            if (left && !right) {
                left_streak = (left_streak < 255) ? (left_streak + 1) : left_streak;
                right_streak = 0;
            } else if (!left && right) {
                right_streak = (right_streak < 255) ? (right_streak + 1) : right_streak;
                left_streak = 0;
            } else {
                // Both or none: don't adjust aggressively.
                left_streak = 0;
                right_streak = 0;
            }

            if (left_streak >= confirm_streak && left && !right) {
                target_angle = clamp_angle(target_angle - track_step, min_angle, max_angle);
            } else if (right_streak >= confirm_streak && !left && right) {
                target_angle = clamp_angle(target_angle + track_step, min_angle, max_angle);
            }

            servo_set_angle(SERVO_SCAN_LEFT, target_angle);
            // Mirror mapping must match scanning mode.
            servo_set_angle(SERVO_SCAN_RIGHT, max_angle - (target_angle - min_angle));
            servo_set_angle(SERVO_WATER_NOZZLE, target_angle);

            if (hc_sr04_is_target_near(stop_distance_cm)) {
                motor_stop();
                ESP_LOGI(TAG, "Reached target (%.2f cm). Stop and spray.", hc_sr04_get_last_distance_cm());
            } else {
                motor_forward(700);
            }

            // Reset lock if flame disappears on both sensors.
            if (!left && !right) {
                ESP_LOGI(TAG, "Flame lost, wait 3s before resume scanning");
                vTaskDelay(unlock_delay_ticks);
                left = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
                right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
                if (!left && !right) {
                    locked = false;
                    fire_confirmed = false;
                    buzzer_update(false);
                    motor_stop();
                    ESP_LOGI(TAG, "Flame confirmed lost, resume scanning");
                } else {
                    fire_confirmed = true;
                    ESP_LOGI(TAG, "Flame detected again during wait, keep tracking");
                }
            } else {
                TickType_t now = xTaskGetTickCount();
                if (now - last_track_log_ticks >= track_log_period_ticks) {
                    last_track_log_ticks = now;
                    if (left && right) {
                        ESP_LOGI(TAG, "Tracking flame center at %.1f deg", target_angle);
                    } else if (left) {
                        ESP_LOGI(TAG, "Tracking flame LEFT -> %.1f deg", target_angle);
                    } else {
                        ESP_LOGI(TAG, "Tracking flame RIGHT -> %.1f deg", target_angle);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    motor_init();
    servo_init();
    buzzer_init();
    frame_sensor_init();
    frame_sensor_start_monitoring(300);
    hc_sr04_init();
    hc_sr04_start_monitoring(200);

    xTaskCreate(fire_tracking_task, "fire_tracking", 4096, NULL, 6, NULL);
}
