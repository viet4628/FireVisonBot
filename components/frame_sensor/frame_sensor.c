#include "frame_sensor.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FLAME_LEFT_GPIO  GPIO_NUM_4
#define FLAME_RIGHT_GPIO GPIO_NUM_6

static const char *TAG = "FLAME";
static bool monitor_started = false;

void frame_sensor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLAME_LEFT_GPIO) | (1ULL << FLAME_RIGHT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static gpio_num_t flame_gpio_from_id(flame_sensor_id_t id)
{
    return (id == FLAME_SENSOR_RIGHT) ? FLAME_RIGHT_GPIO : FLAME_LEFT_GPIO;
}

bool frame_sensor_is_fire_detected(flame_sensor_id_t id)
{
    /* Cảm biến mặc định xuất mức THẤP (0) khi phát hiện lửa, và CAO (1) khi bình thường. */
    return gpio_get_level(flame_gpio_from_id(id)) == 0;
}

bool frame_sensor_any_fire_detected(void)
{
    return frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT) ||
           frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
}

static void frame_sensor_monitor_task(void *arg)
{
    uint32_t period_ms = (uint32_t)(uintptr_t)arg;

    while (1) {
        bool left = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
        bool right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
        int raw_L = gpio_get_level(FLAME_LEFT_GPIO);
        int raw_R = gpio_get_level(FLAME_RIGHT_GPIO);

        if (left && right) {
            ESP_LOGI(TAG, "FIRE: BOTH (RAW: L=%d R=%d)", raw_L, raw_R);
        } else if (left) {
            ESP_LOGI(TAG, "FIRE: LEFT (RAW: L=%d R=%d)", raw_L, raw_R);
        } else if (right) {
            ESP_LOGI(TAG, "FIRE: RIGHT (RAW: L=%d R=%d)", raw_L, raw_R);
        } else {
            /* Khi không có lửa cũng in ra tín hiệu RAW mỗi 2 giây để dễ debug */
            static int no_fire_count = 0;
            if (++no_fire_count % 4 == 0) {
                // ESP_LOGI(TAG, "FIRE: NONE (RAW: L=%d R=%d)", raw_L, raw_R);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void frame_sensor_start_monitoring(uint32_t period_ms)
{
    if (period_ms == 0) {
        period_ms = 500;
    }

    if (monitor_started) {
        return;
    }

    monitor_started = true;
    xTaskCreate(
        frame_sensor_monitor_task,
        "frame_sensor",
        2048,
        (void *)(uintptr_t)period_ms,
        5,
        NULL
    );
}
