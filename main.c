#include <stdio.h>
#include "prototype3.h"

void app_main(void)
{
    setup_gpio();

    xTaskCreate(echo_sensor, "Ultrasonic Task", 2048, NULL, 5, NULL);
    static bool is_ledc_initialized = false;
    if (!is_ledc_initialized) {
        pwm_init();
        is_ledc_initialized = true;
    }

    while (1) {

        motor_controller();
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Add a delay to avoid busy-waiting
    }

}