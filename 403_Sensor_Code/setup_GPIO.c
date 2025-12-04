#include "prototype4.h"



// Function to initialize GPIO
void init_gpio() {
    
    int pins[] = {TRANS1, TRANS2, TRANS3, TRANS4, TRANS5, TRANS6, TRANS7, TRANS8};
    for (int i = 0; i < 8; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);  // Default to OFF
    }
    int pins1[] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5, ECHO6};
    int trigg[] = {TRIGGER1, TRIGGER2, TRIGGER3, TRIGGER4, TRIGGER5, TRIGGER6};
    
    for (int i = 0; i < 6; i++) {
        gpio_reset_pin(pins1[i]);
        gpio_set_direction(pins1[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(pins1[i], GPIO_PULLUP_DISABLE);  // disable internal pullups/pulldowns
    
        gpio_reset_pin(trigg[i]);
        gpio_set_direction(trigg[i], GPIO_MODE_OUTPUT);
        gpio_set_level(trigg[i], 0);
    }
    

}

void init_adc() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(JOYSTICK_UD, ADC_ATTEN);
    adc1_config_channel_atten(JOYSTICK_LR, ADC_ATTEN);
}

int calibrate_joystick(adc_channel_t channel) {
    int total = 0;
    const int num_samples = 10;  // Number of samples to take for calibration
    for (int i = 0; i < num_samples; i++) {
        total += read_joystick(channel);
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between reads to allow ADC to stabilize
    }
    return total / num_samples;  // Return average value
}


int read_joystick(adc_channel_t channel) {
    int raw_value = adc1_get_raw(channel);  // Read raw ADC value from the joystick
    int voltage = (raw_value * VREF) / ADC_MAX_VALUE;  // Convert to mV (millivolts)
    printf("Voltage =%d\n", voltage);
    return voltage;  // Return the voltage value
}