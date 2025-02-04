#include "prototype1.h"

void trigger_sensor(float *distance) {
    int64_t start_time, end_time;

    // Send 10µs trigger pulse
    gpio_set_level(TRIGGER1, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIGGER1, 0);

    // Wait for echo start (rising edge)
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO1) == 0) {
        if (esp_timer_get_time() - start_time > MAX_ECHO_TIMEOUT_US) {
            ESP_LOGE(TAG, "Ping timeout (no echo detected)");
            *distance = -1;
            return;
        }
    }

    // Measure echo pulse width (time until falling edge)
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO1) == 1) {
        if (esp_timer_get_time() - start_time > MAX_ECHO_TIMEOUT_US) {
            ESP_LOGE(TAG, "Echo timeout (distance too large)");
            *distance = -1;
            return;
        }
    }
    end_time = esp_timer_get_time();

    // Calculate distance in cm using the given formula
    int64_t pulse_duration = end_time - start_time;
    *distance = (float)pulse_duration / 58.0f;  // Distance in cm

    ESP_LOGI(TAG, "Distance: %.2f cm", *distance);
}

