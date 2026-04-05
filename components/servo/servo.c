#include "servo.h"
#include "board_hw.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdbool.h>

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MIN_DEGREE        (-15)
#define SERVO_MAX_DEGREE        180

#define SERVO_FREQ_HZ           BOARD_SERVO_LEDC_FREQ_HZ
#define SERVO_DUTY_RES          BOARD_SERVO_LEDC_TIMER_BITS
#define SERVO_DUTY_MAX          BOARD_SERVO_DUTY_MAX_FLOAT

typedef struct {
    ledc_channel_t channel;
    float current_angle;
    bool initialized;
} servo_state_t;

static servo_state_t servos[SERVO_COUNT] = {0};

static uint32_t angle_to_duty(float angle)
{
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    uint32_t pulse_us = SERVO_MIN_PULSEWIDTH_US +
                        ((angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
                         (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE));

    uint32_t duty = (uint32_t)((float)pulse_us * SERVO_DUTY_MAX / 20000.0f);
    return duty;
}

void servo_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = BOARD_SERVO_LEDC_TIMER,
        .duty_resolution  = SERVO_DUTY_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    int pins[SERVO_COUNT] = {
        BOARD_GPIO_SERVO_SCAN_LEFT,
        BOARD_GPIO_SERVO_SCAN_RIGHT,
        BOARD_GPIO_SERVO_FPV_PAN,
        BOARD_GPIO_SERVO_FPV_TILT,
    };
    ledc_channel_t channels[SERVO_COUNT] = {
        BOARD_SERVO_CH_SCAN_LEFT,
        BOARD_SERVO_CH_SCAN_RIGHT,
        BOARD_SERVO_CH_FPV_PAN,
        BOARD_SERVO_CH_FPV_TILT,
    };

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = channels[i],
            .timer_sel      = BOARD_SERVO_LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = pins[i],
            .duty           = 0,
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);

        servos[i].channel = channels[i];
        servos[i].initialized = true;

        servo_set_angle((servo_id_t)i, 90.0f);
    }
}

void servo_set_angle(servo_id_t id, float angle)
{
    if (id < 0 || id >= SERVO_COUNT || !servos[id].initialized) return;

    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[id].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[id].channel);

    servos[id].current_angle = angle;
}

float servo_get_angle(servo_id_t id)
{
    if (id < 0 || id >= SERVO_COUNT || !servos[id].initialized) return 0.0f;
    return servos[id].current_angle;
}

void servo_set_primary_angle(float angle)
{
    servo_set_angle(SERVO_SCAN_LEFT, angle);
}

static void servo_sweep_task(void *arg)
{
    const float step = 1.5f;
    (void)arg;
    while (1) {
        for (float angle = SERVO_MIN_DEGREE; angle <= SERVO_MAX_DEGREE; angle += step) {
            servo_set_angle(SERVO_SCAN_LEFT, angle);
            servo_set_angle(SERVO_SCAN_RIGHT, SERVO_MAX_DEGREE - (angle - SERVO_MIN_DEGREE));
            vTaskDelay(pdMS_TO_TICKS(15));
        }
        vTaskDelay(pdMS_TO_TICKS(150));
        for (float angle = SERVO_MAX_DEGREE; angle >= SERVO_MIN_DEGREE; angle -= step) {
            servo_set_angle(SERVO_SCAN_LEFT, angle);
            servo_set_angle(SERVO_SCAN_RIGHT, SERVO_MAX_DEGREE - (angle - SERVO_MIN_DEGREE));
            vTaskDelay(pdMS_TO_TICKS(15));
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void servo_sweep_start(void)
{
    xTaskCreate(servo_sweep_task, "servo_sweep", 2048, NULL, 5, NULL);
}
