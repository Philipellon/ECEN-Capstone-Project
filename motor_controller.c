#include "prototype1.h"


void motor_controller(void) {
    // Read the state of switch 1 and switch 2
    int switch1_state = gpio_get_level(SWITCH1);
    int switch2_state = gpio_get_level(SWITCH2);

    if (switch1_state == 0 && switch2_state == 1) { // Switch1 pressed, Switch2 not pressed
        char *if1 = pcTaskGetName(NULL);
        ESP_LOGI(if1, "Switch1 pressed, Switch2 not pressed\n");
        gpio_set_level(TRANS1, 0); // Trans1 offThe server is busy. Please try again later.
        gpio_set_level(TRANS2, 1); // Trans2 on
        gpio_set_level(TRANS3, 0); // Trans3 off
        gpio_set_level(TRANS4, 1); // Trans4 on
    } else if (switch2_state == 0 && switch1_state == 1) { // Switch2 pressed, Switch1 not pressed
        char *if2 = pcTaskGetName(NULL);
        ESP_LOGI(if2, "Switch2 pressed, Switch1 not pressed\n");
        gpio_set_level(TRANS1, 1); // Trans1 on
        gpio_set_level(TRANS2, 0); // Trans2 off
        gpio_set_level(TRANS3, 1); // Trans3 on
        gpio_set_level(TRANS4, 0); // Trans4 off
    } else { // Neither switch is pressed (or both are pressed)
        char *if3 = pcTaskGetName(NULL);
        ESP_LOGI(if3, "No switch pressed (or both pressed)\n");
        gpio_set_level(TRANS1, 0); // Trans1 off
        gpio_set_level(TRANS2, 0); // Trans2 off
        gpio_set_level(TRANS3, 0); // Trans3 off
        gpio_set_level(TRANS4, 0); // Trans4 off
    }
}
