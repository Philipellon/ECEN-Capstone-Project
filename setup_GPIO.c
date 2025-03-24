#include "prototype4.h"

// Joystick Calibration
int UPMID = 0;   // Center position for Up/Down
int LRMID = 0;   // Center position for Left/Right
int MID_TOLERANCE = 10;  // Dead zone threshold

// Function to initialize GPIO
void init_gpio() {
    int pins[] = {TRANS1, TRANS2, TRANS3, TRANS4, TRANS5, TRANS6, TRANS7, TRANS8};
    for (int i = 0; i < 8; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);  // Default to OFF
    }
}

// Function to read joystick values
int read_joystick(adc_channel_t channel) {
    int raw_value = adc1_get_raw(channel);
    return raw_value;
}

// Function to calibrate joystick center
void calibrate_joystick() {
    UPMID = read_joystick(JOYSTICK_UD);
    LRMID = read_joystick(JOYSTICK_LR);
}