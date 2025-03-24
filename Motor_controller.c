#include "prototype4.h"



void motor_controller(){
        int UD = read_joystick(JOYSTICK_UD);
        int LR = read_joystick(JOYSTICK_LR);

        // Default: Stop both motors
        int left_forward = 0, left_reverse = 0;
        int right_forward = 0, right_reverse = 0;

        // **Forward Movement**
        if (UD > UPMID + MID_TOLERANCE) {
            left_forward = 1;
            right_forward = 1;
        }
        // **Backward Movement**
        else if (UD < UPMID - MID_TOLERANCE) {
            left_reverse = 1;
            right_reverse = 1;
        }
        // **Turn Left (Right motor forward, Left motor backward)**
        else if (LR < LRMID - MID_TOLERANCE) {
            left_reverse = 1;
            right_forward = 1;
        }
        // **Turn Right (Left motor forward, Right motor backward)**
        else if (LR > LRMID + MID_TOLERANCE) {
            left_forward = 1;
            right_reverse = 1;
        }
}

// Function to set motor states
void control_wheelchair(int left_forward, int left_reverse, int right_forward, int right_reverse) {
    if(sensor_fault == 1){
        ESP_LOGI(pcTaskGetName(NULL), "Too close to something or ledge Detected!");
        // setting both motors to off
        int left_forward = 0, left_reverse = 0;
        int right_forward = 0, right_reverse = 0;

        gpio_set_level(TRANS1, left_reverse);
        gpio_set_level(TRANS2, left_forward);
        gpio_set_level(TRANS3, left_forward);
        gpio_set_level(TRANS4, left_reverse);
        gpio_set_level(TRANS5, right_reverse);
        gpio_set_level(TRANS6, right_forward);
        gpio_set_level(TRANS7, right_forward);
        gpio_set_level(TRANS8, right_reverse);
    }
    else{
        gpio_set_level(TRANS1, left_reverse);
        gpio_set_level(TRANS2, left_forward);
        gpio_set_level(TRANS3, left_forward);
        gpio_set_level(TRANS4, left_reverse);
        gpio_set_level(TRANS5, right_reverse);
        gpio_set_level(TRANS6, right_forward);
        gpio_set_level(TRANS7, right_forward);
        gpio_set_level(TRANS8, right_reverse);
    }

}
