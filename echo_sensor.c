#include "prototype4.h"
int sensor_fault;
#define NUM_SENSORS 6
#define OBSTACLE_THRESHOLD 35  // Obstacle detection threshold in cm (increase to increase threshold radius around wheelchair)
#define EXPECTED_DOWNWARD 25.4
#define DOWNWARD_TOLERANCE 5.0
#define FAULT_THRESHOLD 2


void echo_sensor(void *pvParameter) {
    int trigPins[NUM_SENSORS] = {TRIGGER1, TRIGGER2, TRIGGER3, TRIGGER4, TRIGGER5, TRIGGER6};
    int echoPins[NUM_SENSORS] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5, ECHO6};
    int too_close_count[4] = {0};
    static bool downward_fault = false;  // Persists across iterations
    ESP_LOGW(TAG, "CODE RESET");

    while (1) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            float distance = trigger_sensor(trigPins[i], echoPins[i]);
            ESP_LOGI(TAG, "Sensor %d raw distance: %.2f cm", i + 1, distance);

            if (distance > 0) {
                if (i < 4) {  // Sensors 1–4: Check for obstacles
                    if (distance <= OBSTACLE_THRESHOLD) {
                        too_close_count[i]++;
                        if(too_close_count[i] >= 10){
                            too_close_count[i] = 10;
                        }
                    } else if (too_close_count[i] > 0) {
                        too_close_count[i] -= 2;  // Faster decrement when cleared
                        if (too_close_count[i] < 0) {
                            too_close_count[i] = 0;  // Ensure it doesn't go negative
                        }
                    }
                }
                else {  // Sensors 5–6: Check downward sensor range
                    if (distance < EXPECTED_DOWNWARD - DOWNWARD_TOLERANCE ||
                        distance > EXPECTED_DOWNWARD + DOWNWARD_TOLERANCE) {
                        downward_fault = true;
                        ESP_LOGW(TAG, "Downward sensor %d out of range: %.2f cm", i + 1, distance);
                    } else {
                        downward_fault = false;  // Clear downward fault if back in range
                    }
                }
            } else {
                ESP_LOGW(TAG, "Sensor %d OUT OF RANGE or not working", i + 1);
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Check for sensor faults
        sensor_fault = downward_fault;
        for (int i = 0; i < 4; i++) {
            if (too_close_count[i] >= FAULT_THRESHOLD) {
                sensor_fault = 1;
                break;
            }
        }

        if (sensor_fault) {
            ESP_LOGE(TAG, "SENSOR FAULT TRIGGERED");
        } else {
            ESP_LOGW(TAG, "No Sensor fault");
        }

        ESP_LOGI(TAG, "Fault states: Too close counts [%d, %d, %d, %d], Downward fault: %d",
                 too_close_count[0], too_close_count[1], too_close_count[2], too_close_count[3], downward_fault);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


