#include "servo.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===== CONFIG =====
#define SERVO_MIN_PULSEWIDTH_US 600
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE        0
#define SERVO_MAX_DEGREE        180

#define SERVO_GPIO              14
#define SERVO_RESOLUTION_HZ     1000000
#define SERVO_PERIOD            20000

// ===== HANDLES =====
static mcpwm_cmpr_handle_t comparator = NULL;

// Convert angle → pulse
static inline uint32_t angle_to_compare(float angle)
{
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    return SERVO_MIN_PULSEWIDTH_US +
           (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / SERVO_MAX_DEGREE);
}

// ===== INIT =====
void servo_init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_RESOLUTION_HZ,
        .period_ticks = SERVO_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    mcpwm_new_operator(&operator_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_config, &comparator);

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO,
    };
    mcpwm_new_generator(oper, &generator_config, &generator);

    mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    );

    mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            comparator,
            MCPWM_GEN_ACTION_LOW
        )
    );

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // center
    servo_set_angle(90);
}

// ===== SET ANGLE =====
void servo_set_angle(float angle)
{
    mcpwm_comparator_set_compare_value(
        comparator,
        angle_to_compare(angle)
    );
}

// ===== SWEEP TASK (OPTIONAL) =====
static void servo_sweep_task(void *arg)
{
    float angle = 90;
    float step = 1.5;

    while (1) {
        servo_set_angle(angle);
        vTaskDelay(pdMS_TO_TICKS(15));

        angle += step;

        if (angle >= SERVO_MAX_DEGREE || angle <= SERVO_MIN_DEGREE) {
            step = -step;
        }
    }
}

void servo_sweep_start(void)
{
    xTaskCreate(servo_sweep_task, "servo_sweep", 2048, NULL, 5, NULL);
}