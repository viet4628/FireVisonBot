#include "servo.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#define SERVO_MIN_PULSEWIDTH_US 450
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE        -15
#define SERVO_MAX_DEGREE        180

/* ---------- GPIO assignment ---------- */
#define SERVO_SCAN_LEFT_GPIO    15
#define SERVO_SCAN_RIGHT_GPIO   21
#define SERVO_FPV_PAN_GPIO      38
#define SERVO_FPV_TILT_GPIO     39

#define SERVO_RESOLUTION_HZ     1000000
#define SERVO_PERIOD            20000

typedef struct {
    mcpwm_cmpr_handle_t comparator;
    float current_angle;
    bool initialized;
} servo_channel_t;

static servo_channel_t servos[SERVO_COUNT] = {0};

static inline uint32_t angle_to_compare(float angle)
{
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    return SERVO_MIN_PULSEWIDTH_US +
           ((angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
            (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE));
}

static int servo_gpio_from_id(servo_id_t id)
{
    switch (id) {
        case SERVO_SCAN_LEFT:  return SERVO_SCAN_LEFT_GPIO;
        case SERVO_SCAN_RIGHT: return SERVO_SCAN_RIGHT_GPIO;
        case SERVO_FPV_PAN:    return SERVO_FPV_PAN_GPIO;
        case SERVO_FPV_TILT:   return SERVO_FPV_TILT_GPIO;
        default: return -1;
    }
}

/*
 * ESP32-S3 has 2 MCPWM groups (0 and 1), each with 3 timers/operators.
 * Distribute 4 servos across 2 groups, max 3 per group:
 *   Group 0 : SCAN_LEFT, FPV_PAN     (2 units)
 *   Group 1 : SCAN_RIGHT, FPV_TILT   (2 units)
 */
static int servo_group_from_id(servo_id_t id)
{
    switch (id) {
        case SERVO_SCAN_LEFT:  return 0;
        case SERVO_SCAN_RIGHT: return 1;
        case SERVO_FPV_PAN:    return 0;
        case SERVO_FPV_TILT:   return 1;
        default: return 0;
    }
}

static void servo_init_one(servo_id_t id)
{
    int gpio = servo_gpio_from_id(id);
    if (gpio < 0) {
        return;
    }

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = servo_group_from_id(id),
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_RESOLUTION_HZ,
        .period_ticks = SERVO_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = servo_group_from_id(id),
    };
    mcpwm_new_operator(&operator_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_config, &servos[id].comparator);

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio,
    };
    mcpwm_new_generator(oper, &generator_config, &generator);

    mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH));

    mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            servos[id].comparator,
            MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    servos[id].initialized = true;
    servo_set_angle(id, 90.0f);
}

void servo_init(void)
{
    for (int i = 0; i < SERVO_COUNT; ++i) {
        servo_init_one((servo_id_t)i);
    }
}

void servo_set_angle(servo_id_t id, float angle)
{
    if (id < 0 || id >= SERVO_COUNT || !servos[id].initialized) {
        return;
    }

    mcpwm_comparator_set_compare_value(servos[id].comparator, angle_to_compare(angle));
    servos[id].current_angle = angle;
}

float servo_get_angle(servo_id_t id)
{
    if (id < 0 || id >= SERVO_COUNT || !servos[id].initialized) {
        return 0.0f;
    }
    return servos[id].current_angle;
}

void servo_set_primary_angle(float angle)
{
    servo_set_angle(SERVO_SCAN_LEFT, angle);
}

static void servo_sweep_task(void *arg)
{
    (void)arg;
    const float step = 1.5f;

    while (1) {
        for (float angle = SERVO_MIN_DEGREE; angle < SERVO_MAX_DEGREE; angle += step) {
            servo_set_angle(SERVO_SCAN_LEFT, angle);
            servo_set_angle(SERVO_SCAN_RIGHT, SERVO_MAX_DEGREE - (angle - SERVO_MIN_DEGREE));
            vTaskDelay(pdMS_TO_TICKS(15));
        }

        servo_set_angle(SERVO_SCAN_LEFT, SERVO_MAX_DEGREE);
        servo_set_angle(SERVO_SCAN_RIGHT, SERVO_MIN_DEGREE);
        vTaskDelay(pdMS_TO_TICKS(150));

        for (float angle = SERVO_MAX_DEGREE; angle > SERVO_MIN_DEGREE; angle -= step) {
            servo_set_angle(SERVO_SCAN_LEFT, angle);
            servo_set_angle(SERVO_SCAN_RIGHT, SERVO_MAX_DEGREE - (angle - SERVO_MIN_DEGREE));
            vTaskDelay(pdMS_TO_TICKS(15));
        }

        servo_set_angle(SERVO_SCAN_LEFT, SERVO_MIN_DEGREE);
        servo_set_angle(SERVO_SCAN_RIGHT, SERVO_MAX_DEGREE);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void servo_sweep_start(void)
{
    xTaskCreate(servo_sweep_task, "servo_sweep", 3072, NULL, 5, NULL);
}