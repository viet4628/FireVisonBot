#include "motor.h"
#include "board_hw.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <stddef.h>

#define PWM_GPIO1    BOARD_GPIO_MOTOR_PWM_LEFT
#define RPWM_GPIO1   BOARD_GPIO_MOTOR_RPWM_LEFT
#define LPWM_GPIO1   BOARD_GPIO_MOTOR_LPWM_LEFT
#define PWM_GPIO2    BOARD_GPIO_MOTOR_PWM_RIGHT
#define RPWM_GPIO2   BOARD_GPIO_MOTOR_RPWM_RIGHT
#define LPWM_GPIO2   BOARD_GPIO_MOTOR_LPWM_RIGHT

#define LEDC_FREQ       BOARD_MOTOR_LEDC_FREQ_HZ
#define LEDC_RESOLUTION BOARD_MOTOR_LEDC_TIMER_BITS
#define DUTY_MAX        BOARD_MOTOR_DUTY_MAX

#define MOTOR_TIMER     BOARD_MOTOR_LEDC_TIMER
#define MOTOR1_CHANNEL  BOARD_MOTOR_LEDC_CH_LEFT
#define MOTOR2_CHANNEL  BOARD_MOTOR_LEDC_CH_RIGHT

static void motor_set_all_pins_drive_strong(void)
{
    const gpio_num_t pins[] = {
        PWM_GPIO1, RPWM_GPIO1, LPWM_GPIO1,
        PWM_GPIO2, RPWM_GPIO2, LPWM_GPIO2,
    };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_set_drive_capability(pins[i], GPIO_DRIVE_CAP_3);
    }
}

void motor_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = MOTOR_TIMER,
        .freq_hz         = LEDC_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch1 = {
        .gpio_num   = PWM_GPIO1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = MOTOR1_CHANNEL,
        .timer_sel  = MOTOR_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ch1);

    ledc_channel_config_t ch2 = {
        .gpio_num   = PWM_GPIO2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = MOTOR2_CHANNEL,
        .timer_sel  = MOTOR_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ch2);

    gpio_set_direction(RPWM_GPIO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LPWM_GPIO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RPWM_GPIO2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LPWM_GPIO2, GPIO_MODE_OUTPUT);
    motor_set_all_pins_drive_strong();

    motor_stop();
}

void motor_forward(uint32_t duty)
{
    if (duty > DUTY_MAX) duty = DUTY_MAX;

    gpio_set_level(RPWM_GPIO1, 1);
    gpio_set_level(LPWM_GPIO1, 0);
    gpio_set_level(RPWM_GPIO2, 1);
    gpio_set_level(LPWM_GPIO2, 0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL);
}

void motor_backward(uint32_t duty)
{
    if (duty > DUTY_MAX) duty = DUTY_MAX;

    gpio_set_level(RPWM_GPIO1, 0);
    gpio_set_level(LPWM_GPIO1, 1);
    gpio_set_level(RPWM_GPIO2, 0);
    gpio_set_level(LPWM_GPIO2, 1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL);
}

void motor_turn_left(uint32_t duty)
{
    if (duty > DUTY_MAX) duty = DUTY_MAX;

    gpio_set_level(RPWM_GPIO1, 0);
    gpio_set_level(LPWM_GPIO1, 1);
    gpio_set_level(RPWM_GPIO2, 1);
    gpio_set_level(LPWM_GPIO2, 0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL);
}

void motor_turn_right(uint32_t duty)
{
    if (duty > DUTY_MAX) duty = DUTY_MAX;

    gpio_set_level(RPWM_GPIO1, 1);
    gpio_set_level(LPWM_GPIO1, 0);
    gpio_set_level(RPWM_GPIO2, 0);
    gpio_set_level(LPWM_GPIO2, 1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL);
}

void motor_stop(void)
{
    gpio_set_level(RPWM_GPIO1, 0);
    gpio_set_level(LPWM_GPIO1, 0);
    gpio_set_level(RPWM_GPIO2, 0);
    gpio_set_level(LPWM_GPIO2, 0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL);
}

void motor_drive_curve(uint32_t left_duty, uint32_t right_duty)
{
    if (left_duty  > DUTY_MAX) left_duty  = DUTY_MAX;
    if (right_duty > DUTY_MAX) right_duty = DUTY_MAX;

    /* Cả hai bánh đều tiến — chỉ duty khác nhau để quẹo cong */
    gpio_set_level(RPWM_GPIO1, 1);
    gpio_set_level(LPWM_GPIO1, 0);
    gpio_set_level(RPWM_GPIO2, 1);
    gpio_set_level(LPWM_GPIO2, 0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL, left_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL, right_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_CHANNEL);
}
