#include "prototype4.h"

void trigger_sensor(int trigger_pin, int echo_pin, float *distance) {
    int64_t start_time, end_time;

    // Ensure no signal is sent before triggering
    gpio_set_level(trigger_pin, 0);
    esp_rom_delay_us(2);

    // Send 10µs trigger pulse
    gpio_set_level(trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trigger_pin, 0);

    // Wait for echo start (rising edge)
    start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() - start_time > MAX_ECHO_TIMEOUT_US) {
            ESP_LOGE(TAG, "Sensor %d: Ping timeout (no echo detected)", echo_pin);
            *distance = -1;
            return;
        }
    }

    // Measure echo pulse width (time until falling edge)
    start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() - start_time > MAX_ECHO_TIMEOUT_US) {
            ESP_LOGE(TAG, "Sensor %d: Echo timeout (distance too large)", echo_pin);
            *distance = -1;
            return;
        }
    }
    end_time = esp_timer_get_time();

    // Calculate distance in cm
    int64_t pulse_duration = end_time - start_time;
    *distance = (float)pulse_duration / 58.0f;  // Convert to cm

    ESP_LOGI(TAG, "Sensor %d - Distance: %.2f cm", echo_pin, *distance);
}

