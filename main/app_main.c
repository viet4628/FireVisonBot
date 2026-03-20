#include "motor.h"
#include "servo.h"
#include "frame_sensor.h"
#include "hc_sr04.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    motor_init();
    servo_init();
    frame_sensor_init();
    frame_sensor_start_monitoring(500);
    hc_sr04_init();
    hc_sr04_start_monitoring(200);
    servo_sweep_start();
    while (1) {
        motor_forward(800);
        vTaskDelay(pdMS_TO_TICKS(3000));

        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));

        motor_backward(800);

        vTaskDelay(pdMS_TO_TICKS(3000));
        
    }
}
