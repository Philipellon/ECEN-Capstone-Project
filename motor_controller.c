#include "prototype3.h"

void motor_controller(void) {
    // Read switch states
    int switch1_state = gpio_get_level(SWITCH1);
    int switch2_state = gpio_get_level(SWITCH2);

    if (switch1_state == 0 && switch2_state == 1) { 
        // Switch1 pressed → Forward Motion
        ESP_LOGI(pcTaskGetName(NULL), "Switch1 pressed, running motor forward with PWM");

        // Turn ON high-side PNP (TRANS1)
        gpio_set_level(TRANS1, 0);  // Active LOW for PNP (ON)
        gpio_set_level(TRANS2, 1);  // Ensure TRANS2 is OFF PNP
        gpio_set_level(TRANS3, 0);  // NPN transistor off
        gpio_set_level(TRANS4, 1);  // NPN transistor on

        // Apply PWM to TRANS4 (NPN - Low Side)
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_FORWARD, MOTOR_SPEED_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_FORWARD));

        // Ensure TRANS3 is OFF
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_REVERSE, 0));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_REVERSE));

    } else if (switch2_state == 0 && switch1_state == 1) { 
        // Switch2 pressed → Reverse Motion
        ESP_LOGI(pcTaskGetName(NULL), "Switch2 pressed, running motor reverse with PWM");

        // Turn ON high-side PNP (TRANS2)
        gpio_set_level(TRANS1, 1);  // Active LOW for PNP (off)
        gpio_set_level(TRANS2, 0);  // Ensure TRANS2 is PNP is on 

        // Apply PWM to TRANS3 (NPN - Low Side)
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_REVERSE, MOTOR_SPEED_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_REVERSE));

        // Ensure TRANS4 is OFF
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_FORWARD, 0));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_FORWARD));

    } else {
        // Neither switch is pressed (or both pressed) → STOP the motor
        ESP_LOGI(pcTaskGetName(NULL), "No switch pressed (or both pressed), stopping motor");

        // Turn OFF both PNP transistors
        gpio_set_level(TRANS1, 1);  // PNP OFF
        gpio_set_level(TRANS2, 1);  // PNP OFF

        // Set both PWM channels to 0 (stop current flow)
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_FORWARD, 0));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_FORWARD));
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_REVERSE, 0));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_REVERSE));
    }
}