#include "frame_sensor.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FLAME_GPIO GPIO_NUM_4

static const char *TAG = "FLAME";
static bool monitor_started = false;

void frame_sensor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLAME_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

bool frame_sensor_is_fire_detected(void)
{
    return gpio_get_level(FLAME_GPIO) == 0;
}

static void frame_sensor_monitor_task(void *arg)
{
    uint32_t period_ms = (uint32_t)(uintptr_t)arg;

    while (1) {
        if (frame_sensor_is_fire_detected()) {
            ESP_LOGI(TAG, "🔥 Fire detected!");
        } else {
            ESP_LOGI(TAG, "No fire");
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
