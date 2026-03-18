#include "motor.h"
#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    motor_init();
    servo_init();
    while (1) {
        motor_forward(800);
        servo_set_angle(30);
        vTaskDelay(pdMS_TO_TICKS(3000));

        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));

        motor_backward(800);
        servo_set_angle(150);

        vTaskDelay(pdMS_TO_TICKS(3000));
        
    }
}
