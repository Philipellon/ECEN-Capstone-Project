#include "prototype3.h"

#define ALPHA 0.2  // Smoothing factor (Adjust between 0.1 - 0.9 for responsiveness)

void echo_sensor(void *pvParameter) {
    float distance;
    float ema_distance = 0;  // Initialize EMA with 0 or the first valid reading

    bool first_reading = true;  // Flag to handle first valid reading

    while (true) {
        trigger_sensor(&distance);

        if (distance > 0) {  // Ignore invalid readings (-1)
            if (first_reading) {
                ema_distance = distance;  // Initialize with first valid reading
                first_reading = false;
            } else {
                ema_distance = (ALPHA * distance) + ((1 - ALPHA) * ema_distance);
            }

            ESP_LOGI(TAG, "EMA Distance: %.2f cm", ema_distance);
        }

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_CYCLE_MS));  // Avoid overlapping signals
    }
}

