#include "prototype4.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us()

float trigger_sensor(gpio_num_t trigger_pin, gpio_num_t echo_pin) {
    int64_t start_time = 0, end_time = 0;

    // Ensure no signal is sent before triggering
    gpio_set_level(trigger_pin, 0);
    esp_rom_delay_us(12);

    // Send 10 µs trigger pulse
    gpio_set_level(trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trigger_pin, 0);

    // 30 ms overall timeout
    uint64_t timeout = esp_timer_get_time() + 30000;

    // Wait for echo to go HIGH (start)
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    start_time = esp_timer_get_time();

    // Wait for echo to go LOW (end)
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    end_time = esp_timer_get_time();

    int64_t pulse_us = end_time - start_time;
    if (pulse_us <= 0) return -1.0f;

    // HC-SR04 style conversion (≈58 µs per cm)
    return (float)pulse_us / 58.0f; // cm
}
