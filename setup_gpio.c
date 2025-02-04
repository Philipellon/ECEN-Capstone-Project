#include "prototype1.h"

void setup_gpio(void) {
    char *ourTaskName = pcTaskGetName(NULL);
    ESP_LOGI(ourTaskName, "Setting up GPIO pins...\n");

    // Reset GPIO pins for transistors
    gpio_reset_pin(TRANS1);
    gpio_reset_pin(TRANS2);
    gpio_reset_pin(TRANS3);
    gpio_reset_pin(TRANS4);
    gpio_reset_pin(SWITCH1);
    gpio_reset_pin(SWITCH2);
    gpio_reset_pin(TRIGGER1);
    gpio_reset_pin(ECHO1);

    // Set transistor pins as output
    gpio_set_direction(TRANS1, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS2, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS3, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS4, GPIO_MODE_OUTPUT);
    


    // Configure switch pins as input with internal pull-up resistors
    //set motor control pins
    gpio_set_pull_mode(SWITCH1, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH2, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH2, GPIO_MODE_INPUT);

    // set ultrasoncic sensor pins
    gpio_set_direction(TRIGGER1, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIGGER1, 0);
    gpio_set_direction(ECHO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO1, GPIO_FLOATING);


}
