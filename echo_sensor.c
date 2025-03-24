#include "prototype4.h"

#define ALPHA 0.2  // Smoothing factor (Adjust between 0.1 - 0.9 for responsiveness)
#define SENSOR_DELAY_MS 10  // Delay to avoid interference

void echo_sensor(void *pvParameter) {
    float distances[6];   // Raw distances
    float ema_distances[6] = {0};  // Smoothed distances
    bool first_reading[6] = {true}; // First-time flags

    int triggers[6] = {TRIGGER1, TRIGGER2, TRIGGER1, TRIGGER2, TRIGGER1, TRIGGER2};  
    int echoes[6] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5, ECHO6};

    while (true) {
        for (int i = 0; i < 6; i++) {
            trigger_sensor(triggers[i], echoes[i], &distances[i]);  // Trigger one sensor at a time

            if (distances[i] > 0) {  // Ignore invalid readings (-1)
                if (first_reading[i]) {
                    ema_distances[i] = distances[i];  // Initialize with first valid reading
                    first_reading[i] = false;
                } else {
                    ema_distances[i] = (ALPHA * distances[i]) + ((1 - ALPHA) * ema_distances[i]);
                }

                ESP_LOGI(TAG, "Sensor %d - EMA Distance: %.2f cm", i + 1, ema_distances[i]);
            }

            vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY_MS));  // **Delay before triggering next sensor**
        }

        // Ledge Detection Logic (Sensors 5 & 6)
        if (ema_distances[4] >= 20 && ema_distances[5] >= 20) {
            ESP_LOGW(TAG, "Ledge Detected!");
            sensor_fault = 1;
        } else {
            ESP_LOGI(TAG, "Safe - No Ledge");
            sensor_fault = 0;
        }

        // Wheelchair Shutdown Logic (Sensors 1 & 2 )
        if (ema_distances[0] < 20 || ema_distances[1] < 20 || ema_distances[2] < 20 || ema_distances[3] < 20) {
            ESP_LOGE(TAG, "Obstacle too close! Stopping wheelchair...");
            sensor_fault = 1;
        }
        else{
            ESP_LOGI(TAG, "Safe - No Obstacle");
            sensor_fault = 0;
        }
    }
}
