#include <stdio.h>
#include "prototype4.h"
int UPMID = 0;  // Define UPMID as int
int LRMID = 0;  // Define LRMID as int
int sensor_fault = 0;  // Define sensor_fault as int


void app_main(void)
{
    // Initialize GPIO & ADC
    init_gpio();
    init_adc();
    printf("Calibrating joystick...\n");
    UPMID = calibrate_joystick(JOYSTICK_UD);
    LRMID = calibrate_joystick(JOYSTICK_LR);
    printf("Calibration complete: UPMID=%d, LRMID=%d\n", UPMID, LRMID);
    

    xTaskCreate(echo_sensor, "Ultrasonic Task", 2048, NULL, 5, NULL);

    while (1) {
        //call to get motor states and then apply movement
        motor_controller();
        // Delay to stabilize
        vTaskDelay(pdMS_TO_TICKS(100));  
    }

}