#include "hc_sr04.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HC_SR04_TRIG_GPIO  GPIO_NUM_5
#define HC_SR04_ECHO_GPIO  GPIO_NUM_18

#define DIST_THRESHOLD_CM  30.0f
#define MIN_CHANGE_CM      1.0f

static const char *TAG = "HC_SR04";

typedef struct {
    mcpwm_cap_timer_handle_t cap_timer;
    mcpwm_cap_channel_handle_t cap_chan;
    TaskHandle_t waiting_task;
    uint32_t cap_val_begin;
    float last_distance;
    float latest_distance;
    bool started;
} hc_sr04_state_t;

static hc_sr04_state_t sensor = {0};
static bool monitor_started = false;

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan,
                                  const mcpwm_capture_event_data_t *edata,
                                  void *user_data)
{
    (void)cap_chan;
    hc_sr04_state_t *state = (hc_sr04_state_t *)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    if (!state) {
        return false;
    }

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        state->cap_val_begin = edata->cap_value;
    } else {
        uint32_t cap_val_end = edata->cap_value;
        uint32_t tof_ticks = cap_val_end - state->cap_val_begin;

        if (state->waiting_task) {
            xTaskNotifyFromISR(state->waiting_task, tof_ticks,
                               eSetValueWithOverwrite, &high_task_wakeup);
        }
    }

    return high_task_wakeup == pdTRUE;
}

static void hc_sr04_gen_trig_output(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);
}

void hc_sr04_init(void)
{
    if (sensor.started) {
        return;
    }

    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &sensor.cap_timer));

    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(sensor.cap_timer, &cap_ch_conf, &sensor.cap_chan));

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(sensor.cap_chan, &cbs, &sensor));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(sensor.cap_chan));

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);

    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(sensor.cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(sensor.cap_timer));

    sensor.last_distance = 0.0f;
    sensor.latest_distance = 0.0f;
    sensor.started = true;
}

static void hc_sr04_monitor_task(void *arg)
{
    uint32_t period_ms = (uint32_t)(uintptr_t)arg;
    uint32_t tof_ticks;

    if (period_ms == 0) {
        period_ms = 200;
    }

    while (1) {
        sensor.waiting_task = xTaskGetCurrentTaskHandle();
        hc_sr04_gen_trig_output();

        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks,
                            pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0f / esp_clk_apb_freq());

            if (pulse_width_us > 35000.0f) {
                vTaskDelay(pdMS_TO_TICKS(period_ms));
                continue;
            }

            float distance = pulse_width_us / 58.0f;
            sensor.latest_distance = distance;

            if (distance > 0.0f &&
                distance < DIST_THRESHOLD_CM &&
                fabsf(distance - sensor.last_distance) > MIN_CHANGE_CM) {
                ESP_LOGI(TAG, "🚨 Object detected: %.2f cm", distance);
                sensor.last_distance = distance;
            }
        }

        sensor.waiting_task = NULL;
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void hc_sr04_start_monitoring(uint32_t period_ms)
{
    if (!sensor.started) {
        hc_sr04_init();
    }

    if (monitor_started) {
        return;
    }

    monitor_started = true;
    xTaskCreate(hc_sr04_monitor_task, "hc_sr04", 4096, (void *)(uintptr_t)period_ms, 5, NULL);
}

float hc_sr04_get_last_distance_cm(void)
{
    return sensor.latest_distance;
}

bool hc_sr04_is_target_near(float threshold_cm)
{
    float d = sensor.latest_distance;
    return (d > 0.0f) && (d < threshold_cm);
}
