#include <stdio.h>
#include "prototype4.h"

void app_main(void)
{
    // Initialize GPIO & ADC
    init_gpio();
    init_adc();
    int left_forward = 0, left_reverse = 0;
    int right_forward = 0, right_reverse = 0;

    // Calibrate joystick
    calibrate_joystick();

    xTaskCreate(echo_sensor, "Ultrasonic Task", 2048, NULL, 5, NULL);

    while (1) {
        //call to get motor states
        motor_controller();
         // Apply movement
        control_wheelchair(left_forward, left_reverse, right_forward, right_reverse);
        // Delay to stabilize
        vTaskDelay(pdMS_TO_TICKS(100));  
    }

}