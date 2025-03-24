#include "prototype4.h"


void motor_controller(){
    int UD = read_joystick(JOYSTICK_UD);
    int LR = read_joystick(JOYSTICK_LR);
    int MID_TOLERANCE = 100;
    int left_forward; 
    int left_reverse;
    int right_forward; 
    int right_reverse;

    // **Forward Movement**
    if (UD > UPMID + MID_TOLERANCE) {
        left_forward = 1;
        right_forward = 1;
        left_reverse = 0;
        right_reverse = 0;
        }
        // **Backward Movement**
    else if (UD < UPMID - MID_TOLERANCE) {
        left_reverse = 1;
        right_reverse = 1;
        left_forward = 0;
        right_forward = 0;
        }
        // **Turn Left (Right motor forward, Left motor backward)**
    else if (LR < LRMID - MID_TOLERANCE) {
        left_reverse = 1;
        right_forward = 1;
        right_reverse = 0;
        left_forward = 0;
        }
        // **Turn Right (Left motor forward, Right motor backward)**
    else if (LR > LRMID + MID_TOLERANCE) {
        left_forward = 1;
        right_reverse = 1;
        right_forward = 0;
        left_reverse = 0;
        }
    else{
                // Default: Stop both motors
        left_forward = 0;
        left_reverse = 0;
        right_forward = 0;
        right_reverse = 0;
        }

    if(sensor_fault == 1){
        ESP_LOGI(TAG, "Not safe! sensor fault");
        // setting both motors to off
    
        gpio_set_level(TRANS1, 0);
        gpio_set_level(TRANS2, 0);
        gpio_set_level(TRANS3, 0);
        gpio_set_level(TRANS4, 0);
        gpio_set_level(TRANS5, 0);
        gpio_set_level(TRANS6, 0);
        gpio_set_level(TRANS7, 0);
        gpio_set_level(TRANS8, 0);
        }
    else{
        gpio_set_level(TRANS1, left_reverse);
        gpio_set_level(TRANS3, left_forward);
        gpio_set_level(TRANS2, left_forward);
        gpio_set_level(TRANS4, left_reverse);
        gpio_set_level(TRANS5, right_reverse);
        gpio_set_level(TRANS6, right_forward);
        gpio_set_level(TRANS7, right_forward);
        gpio_set_level(TRANS8, right_reverse);
    }
}