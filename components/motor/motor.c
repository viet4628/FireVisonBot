#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// ===== GPIO CONFIG =====
#define PWM_GPIO1    9
#define RPWM_GPIO1   10
#define LPWM_GPIO1   11

#define PWM_GPIO2    12
#define RPWM_GPIO2   13
#define LPWM_GPIO2   14

// ===== PWM CONFIG =====
#define LEDC_FREQ       5000
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define DUTY_MAX        ((1 << LEDC_RESOLUTION) - 1)

#define MOTOR1_CHANNEL  LEDC_CHANNEL_0
#define MOTOR2_CHANNEL  LEDC_CHANNEL_1

void motor_init(void)
{
    // Timer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = LEDC_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Motor 1
    ledc_channel_config_t ch1 = {
        .gpio_num   = PWM_GPIO1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = MOTOR1_CHANNEL,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch1);

    // Motor 2
    ledc_channel_config_t ch2 = {
        .gpio_num   = PWM_GPIO2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = MOTOR2_CHANNEL,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch2);

    // Direction pins
    gpio_set_direction(RPWM_GPIO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LPWM_GPIO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RPWM_GPIO2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LPWM_GPIO2, GPIO_MODE_OUTPUT);

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