#include "board_hw.h"

#include "esp_log.h"

static const char *TAG = "BOARD_HW";

void board_hw_log_layout(void)
{
    ESP_LOGI(TAG,
             "GPIO: flame L/R=%d,%d | SR04 T/E=%d,%d | relay=%d | buzzer=%d",
             (int)BOARD_GPIO_FLAME_LEFT, (int)BOARD_GPIO_FLAME_RIGHT,
             (int)BOARD_GPIO_SR04_TRIG, (int)BOARD_GPIO_SR04_ECHO,
             (int)BOARD_GPIO_RELAY, (int)BOARD_GPIO_BUZZER);
    ESP_LOGI(TAG,
             "Motor PWM/dir: L %d,%d,%d | R %d,%d,%d | LEDC T%d CH%d,%d @ %d Hz",
             (int)BOARD_GPIO_MOTOR_PWM_LEFT, (int)BOARD_GPIO_MOTOR_RPWM_LEFT,
             (int)BOARD_GPIO_MOTOR_LPWM_LEFT,
             (int)BOARD_GPIO_MOTOR_PWM_RIGHT, (int)BOARD_GPIO_MOTOR_RPWM_RIGHT,
             (int)BOARD_GPIO_MOTOR_LPWM_RIGHT,
             (int)BOARD_MOTOR_LEDC_TIMER, (int)BOARD_MOTOR_LEDC_CH_LEFT,
             (int)BOARD_MOTOR_LEDC_CH_RIGHT, BOARD_MOTOR_LEDC_FREQ_HZ);
    ESP_LOGI(TAG,
             "Servo: %d,%d,%d,%d | LEDC T%d CH0-3 @ %d Hz",
             (int)BOARD_GPIO_SERVO_SCAN_LEFT, (int)BOARD_GPIO_SERVO_SCAN_RIGHT,
             (int)BOARD_GPIO_SERVO_FPV_PAN, (int)BOARD_GPIO_SERVO_FPV_TILT,
             (int)BOARD_SERVO_LEDC_TIMER, BOARD_SERVO_LEDC_FREQ_HZ);
}
