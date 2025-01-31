#include <stdio.h>
#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <portmacro.h>
// Define GPIO pins for transistors and switches
#define TRANS1 4  // GPIO pin for transistor 1
#define TRANS2 5  // GPIO pin for transistor 2
#define TRANS3 6  // GPIO pin for transistor 3
#define TRANS4 7  // GPIO pin for transistor 4
#define SWITCH1 8 // GPIO pin for switch 1
#define SWITCH2 9 // GPIO pin for switch 2


void app_main(void) {
    char *ourTaskName = pcTaskGetName(NULL);
    ESP_LOGI(ourTaskName, "Hello, starting up motor control!\n");
    // Reset GPIO pins for transistors (optional but good practice)
    gpio_reset_pin(TRANS1);
    gpio_reset_pin(TRANS2);
    gpio_reset_pin(TRANS3); 
    gpio_reset_pin(TRANS4);
    gpio_reset_pin(SWITCH1);
    gpio_reset_pin(SWITCH2);

    // Set transistor pins as output
    gpio_set_direction(TRANS1, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS2, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS3, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS4, GPIO_MODE_OUTPUT);

    // Configure switch pins as input with internal pull-up resistors
    gpio_set_pull_mode(SWITCH1, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH2, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH2, GPIO_MODE_INPUT);

    // Main loop
    while (true) {
        // Read the state of switch 1
        int switch1_state = gpio_get_level(SWITCH1);
        int switch2_state = gpio_get_level(SWITCH2);

        if (switch1_state == 1 && switch2_state == 0) { // Switch 1 is pressed (active low)
            gpio_set_level(TRANS1, 0); // Turn off transistor 1
            gpio_set_level(TRANS2, 1); // Turn on transistor 2
            gpio_set_level(TRANS3, 0); // Turn off transistor 3
            gpio_set_level(TRANS4, 1); // Turn on transistor 4
        }
        if (switch1_state == 0 && switch2_state == 1) {
            gpio_set_level(TRANS1, 1); // Turn on transistor 1
            gpio_set_level(TRANS2, 0); // Turn off transistor 2
            gpio_set_level(TRANS3, 1); // Turn on transistor 3
            gpio_set_level(TRANS4, 0); // Turn off transistor 4
        } 
        else {
            gpio_set_level(TRANS1, 1); // Turn on transistor 1
            gpio_set_level(TRANS2, 1); // Turn on transistor 2
            gpio_set_level(TRANS3, 0); // Turn off transistor 3
            gpio_set_level(TRANS4, 0); // Turn off transistor 4 
        }
        // Add a small delay to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

