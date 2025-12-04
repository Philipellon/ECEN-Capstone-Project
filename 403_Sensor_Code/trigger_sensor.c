#include "prototype4.h"

float trigger_sensor(int trigger_pin, int echo_pin) {
    int64_t start_time, end_time;

    // Ensure no signal is sent before triggering
    gpio_set_level(trigger_pin, 0);
    esp_rom_delay_us(12);

    // Send 10µs trigger pulse
    gpio_set_level(trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trigger_pin, 0);

    // Wait for echo to go HIGH
    uint64_t timeout = esp_timer_get_time() + 30000; // 30ms timeout
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) return -1;  // Timeout
    }
    start_time = esp_timer_get_time();

    // Wait for echo to go LOW
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) return -1;  // Timeout
    }
    end_time = esp_timer_get_time();
    // Calculate distance in cm using the given formula
    int64_t pulse_duration = end_time - start_time;
    
    // Calculate distance (speed of sound is ~343 m/s or 0.0343 cm/µs)
    float distance = pulse_duration / 58.0f;  // cm
    return distance;
}

