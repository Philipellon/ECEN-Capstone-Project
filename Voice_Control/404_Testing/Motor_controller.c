#include "C:/Users/Salee/Downloads/Impulse3FlashTest5_404/main/prototype4.h"

void motor_controller() {
    int UD = read_joystick(JOYSTICK_UD);
    int LR = read_joystick(JOYSTICK_LR);
    int MID_TOLERANCE = 300;
    static int count = 0;
    static int last_movement = 0;

    int left_forward = 0, left_reverse = 0, right_forward = 0, right_reverse = 0;
    int movement = 0;

    // **Determine Movement Type**
    if (UD > UPMID + MID_TOLERANCE) {
        left_forward = 1;
        right_reverse = 1;
        movement = 1;
        ESP_LOGI(TAG1, "Input Forward Movement");
    } else if (UD < UPMID - MID_TOLERANCE) {
        left_reverse = 1;
        right_forward = 1;
        movement = 2;
        ESP_LOGI(TAG1, "Input Backward Movement");
    } else if (LR < LRMID - MID_TOLERANCE) {
        left_reverse = 1;
        right_reverse = 1;
        movement = 3;
        ESP_LOGI(TAG1, "Input Left Movement");
    } else if (LR > LRMID + MID_TOLERANCE) {
        left_forward = 1;
        right_forward = 1;
        movement = 4;
        ESP_LOGI(TAG1, "Input Right Movement");
    } else {
        ESP_LOGI(TAG1, "Input NO Movement");
        movement = 0;
    }

    // **Prevent Sudden Reversal by Braking First**
    // this worked good with motors not mounted but could need adjustment once mounted
    if (movement != 0 && movement != last_movement) {
        ESP_LOGI(TAG1, "Pulsed Braking Before Direction Change...");
        
        for (int i = 0; i < 3; i++) {  // Adjust loop count for braking effect
            // Turn ON braking
            gpio_set_level(TRANS1, 1);
            gpio_set_level(TRANS2, 1);
            gpio_set_level(TRANS3, 1);
            gpio_set_level(TRANS4, 1);
            gpio_set_level(TRANS5, 1);
            gpio_set_level(TRANS6, 1);
            gpio_set_level(TRANS7, 1);
            gpio_set_level(TRANS8, 1);
            vTaskDelay(pdMS_TO_TICKS(5));  // Brake ON for 5ms (adjust this for more or less braking )
            
            // Turn OFF braking
            gpio_set_level(TRANS1, 0);
            gpio_set_level(TRANS2, 0);
            gpio_set_level(TRANS3, 0);
            gpio_set_level(TRANS4, 0);
            gpio_set_level(TRANS5, 0);
            gpio_set_level(TRANS6, 0);
            gpio_set_level(TRANS7, 0);
            gpio_set_level(TRANS8, 0);
            vTaskDelay(pdMS_TO_TICKS(10));  // Brake OFF for 10ms (adjust this for more or less braking )
        }
    }

    // **Check if movement is consistent for SET_POINT cycles**
    if (movement != 0) {
        count = (movement == last_movement) ? count + 1 : 1;
    } else {
        count = 0;
    }

    last_movement = movement;

    // **Safety Check - Stop if Sensor Fault Detected**
    if (sensor_fault == 1) {
        ESP_LOGE(TAG1, "Not safe! Sensor fault detected, stopping motors.");
        gpio_set_level(TRANS1, 0);
        gpio_set_level(TRANS2, 0);
        gpio_set_level(TRANS3, 0);
        gpio_set_level(TRANS4, 0);
        gpio_set_level(TRANS5, 0);
        gpio_set_level(TRANS6, 0);
        gpio_set_level(TRANS7, 0);
        gpio_set_level(TRANS8, 0);
    }
    // **Ensure motors stop when no movement is detected**
    else if (movement == 0) {
        ESP_LOGI(TAG1, "Stopping motors (No movement detected)");
        gpio_set_level(TRANS1, 0);
        gpio_set_level(TRANS2, 0);
        gpio_set_level(TRANS3, 0);
        gpio_set_level(TRANS4, 0);
        gpio_set_level(TRANS5, 0);
        gpio_set_level(TRANS6, 0);
        gpio_set_level(TRANS7, 0);
        gpio_set_level(TRANS8, 0);
        count = 0;
    }
    // **Execute Movement Only After Stability**
    else if (count >= SET_POINT) {
        ESP_LOGI(TAG1, "Executing Movement for Wheelchair");

        gpio_set_level(TRANS1, left_reverse);
        gpio_set_level(TRANS3, left_forward);
        gpio_set_level(TRANS2, left_forward);
        gpio_set_level(TRANS4, left_reverse);
        gpio_set_level(TRANS5, right_reverse);
        gpio_set_level(TRANS6, right_forward);
        gpio_set_level(TRANS7, right_forward);
        gpio_set_level(TRANS8, right_reverse);

        ESP_LOGI(TAG1, "GPIO States: TRANS1=%d, TRANS2=%d, TRANS3=%d, TRANS4=%d, TRANS5=%d, TRANS6=%d, TRANS7=%d, TRANS8=%d",
                 left_reverse, left_forward, left_forward, left_reverse, right_reverse, right_forward, right_forward, right_reverse);

        count = SET_POINT;
    }
}
