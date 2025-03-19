#include "prototype3.h"

#include "prototype3.h"

#define ALPHA 0.2  // Smoothing factor

void setup_gpio(void) {
    char *ourTaskName = pcTaskGetName(NULL);
    ESP_LOGI(ourTaskName, "Setting up GPIO pins...\n");

    // Reset GPIO pins
    gpio_reset_pin(TRIGGER1);
    gpio_reset_pin(TRIGGER2);
    gpio_reset_pin(ECHO1);
    gpio_reset_pin(ECHO2);
    gpio_reset_pin(ECHO3);
    gpio_reset_pin(ECHO4);
    gpio_reset_pin(ECHO5);
    gpio_reset_pin(ECHO6);

    // Set trigger pins as output
    gpio_set_direction(TRIGGER1, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIGGER1, 0);
    gpio_set_direction(TRIGGER2, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIGGER2, 0);

    // Set echo pins as input
    gpio_set_direction(ECHO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO1, GPIO_FLOATING);
    gpio_set_direction(ECHO2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO2, GPIO_FLOATING);
    gpio_set_direction(ECHO3, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO3, GPIO_FLOATING);
    gpio_set_direction(ECHO4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO4, GPIO_FLOATING);
    gpio_set_direction(ECHO5, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO5, GPIO_FLOATING);
    gpio_set_direction(ECHO6, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO6, GPIO_FLOATING);
}



void setup_gpio(void) {
    char *ourTaskName = pcTaskGetName(NULL);
    ESP_LOGI(ourTaskName, "Setting up GPIO pins...\n");

    // Reset GPIO pins for transistors
    gpio_reset_pin(TRIGGER1);
    gpio_reset_pin(TRIGGER2);
    gpio_reset_pin(ECHO1);
    gpio_reset_pin(ECHO2);
    gpio_reset_pin(ECHO3);
    gpio_reset_pin(ECHO4);
    gpio_reset_pin(ECHO5);
    gpio_reset_pin(ECHO6);
    gpio_reset_pin(SWITCH1);
    gpio_reset_pin(SWITCH2);


    // Set transistor pins as output
    gpio_set_direction(TRANS1, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS2, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS3, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS4, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS5, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS6, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS7, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRANS8, GPIO_MODE_OUTPUT);
    


    // Configure switch pins as input with internal pull-up resistors
    //set motor control pins
    gpio_set_pull_mode(SWITCH1, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH2, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH3, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH3, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH4, GPIO_PULLUP_ONLY);
    gpio_set_direction(SWITCH4, GPIO_MODE_INPUT);

    // trigger sensor setup
    gpio_set_direction(TRIGGER1, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIGGER1, 0);
    gpio_set_direction(TRIGGER2, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIGGER2, 0);

    // echo pin setup
    gpio_set_direction(ECHO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO1, GPIO_FLOATING);
    gpio_set_direction(ECHO2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO2, GPIO_FLOATING);
    gpio_set_direction(ECHO3, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO3, GPIO_FLOATING);
    gpio_set_direction(ECHO4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO4, GPIO_FLOATING);
    gpio_set_direction(ECHO5, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO5, GPIO_FLOATING);
    gpio_set_direction(ECHO6, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO6, GPIO_FLOATING);

}