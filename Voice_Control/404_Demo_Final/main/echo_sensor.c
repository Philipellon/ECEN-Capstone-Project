#include "prototype4.h"

int sensor_fault;

#define NUM_SENSORS 6
#define OBSTACLE_THRESHOLD 15    // Obstacle detection threshold in cm
#define EXPECTED_DOWNWARD 18    // Expected distance from ground in cm
#define DOWNWARD_TOLERANCE 9
#define FAULT_THRESHOLD 4

void echo_sensor(void *pvParameter) {
    int trigPins[NUM_SENSORS] = {TRIGGER1, TRIGGER2, TRIGGER3, TRIGGER4, TRIGGER5, TRIGGER6};
    int echoPins[NUM_SENSORS] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5, ECHO6};
    int too_close_count[4] = {0};

    // Track faults independently for both downward sensors
    static bool downward_faults[2] = {false, false};

    ESP_LOGW(TAG_SENSOR, "CODE RESET");

    while (1) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            float distance = trigger_sensor(trigPins[i], echoPins[i]);
            //ESP_LOGI(TAG_SENSOR, "Sensor %d raw distance: %.2f cm", i + 1, distance);

            if (distance > 0) {
                if (i < 4) {
                    // Sensors 1–4: Obstacle detection
                    if (distance <= OBSTACLE_THRESHOLD) {
                        too_close_count[i]++;
                        if (too_close_count[i] > FAULT_THRESHOLD) too_close_count[i] = FAULT_THRESHOLD;
                    } else if (too_close_count[i] > 0) {
                        too_close_count[i] -= 2;
                        if (too_close_count[i] < 0) too_close_count[i] = 0;
                    }
                } else {
                    // Sensors 5–6: Downward sensors
                    int idx = i - 4;
                    if (distance < EXPECTED_DOWNWARD - DOWNWARD_TOLERANCE ||
                        distance > EXPECTED_DOWNWARD + DOWNWARD_TOLERANCE) {
                        downward_faults[idx] = true;
                        //ESP_LOGW(TAG1, "Downward sensor %d out of range: %.2f cm", i + 1, distance);
                    } else {
                        downward_faults[idx] = false;
                    }
                }
            } else {
                //ESP_LOGW(TAG_SENSOR, "Sensor %d OUT OF RANGE or not working", i + 1);
            }

            vTaskDelay(pdMS_TO_TICKS(10)); // Delay between each sensor read
        }

        // Evaluate overall fault condition
        sensor_fault = 0;

        // Check downward faults
        if (downward_faults[0] || downward_faults[1]) {
            sensor_fault = 1;
        }

        // Check forward obstacle faults
        for (int i = 0; i < 4; i++) {
            if (too_close_count[i] >= FAULT_THRESHOLD) {
                sensor_fault = 1;
                break;
            }
        }

        if (sensor_fault) {
            //ESP_LOGE(TAG_SENSOR, "SENSOR FAULT TRIGGERED");
        } else {
            //ESP_LOGI(TAG_SENSOR, "No Sensor fault");
        }
        //ESP_LOGE(TAG_SENSOR, "Sensor fault state: [%d]", sensor_fault);
        
        ESP_LOGI(TAG_SENSOR, "Fault states: Too close counts [%d, %d, %d, %d], Downward faults: [%d, %d]",
                 too_close_count[0], too_close_count[1], too_close_count[2], too_close_count[3],
                 downward_faults[0], downward_faults[1]);
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay between each full sensor cycle
    }
}
