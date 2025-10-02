#include "C:/Users/Salee/Downloads/Impulse3FlashTest5_404/main/prototype4.h"
#include <stdbool.h>

// Define the global once (matches `extern int sensor_fault` in prototype4.h)
int sensor_fault = 0;

#define NUM_SENSORS         6
#define OBSTACLE_THRESHOLD  35.0f   // cm
#define EXPECTED_DOWNWARD   25.4f   // cm
#define DOWNWARD_TOLERANCE  5.0f    // cm
#define FAULT_THRESHOLD     2       // counts

static const char *TAG_ECHO = "ECHO";

void echo_sensor(void *pvParameter) {
    (void)pvParameter;

    const gpio_num_t trigPins[NUM_SENSORS] = {
        TRIGGER1, TRIGGER2, TRIGGER3, TRIGGER4, TRIGGER5, TRIGGER6
    };
    const gpio_num_t echoPins[NUM_SENSORS] = {
        ECHO1, ECHO2, ECHO3, ECHO4, ECHO5, ECHO6
    };

    int  too_close_count[4] = {0, 0, 0, 0};
    bool downward_fault = false;

    ESP_LOGW(TAG_ECHO, "CODE RESET");

    for (;;) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            float distance_cm = trigger_sensor(trigPins[i], echoPins[i]);

            if (distance_cm > 0.0f) {
                ESP_LOGD(TAG_ECHO, "Sensor %d: %.2f cm", i + 1, distance_cm);

                if (i < 4) {
                    // Front/side obstacle sensors
                    if (distance_cm <= OBSTACLE_THRESHOLD) {
                        if (too_close_count[i] < 10) too_close_count[i]++;
                    } else if (too_close_count[i] > 0) {
                        too_close_count[i] -= 2;     // quicker recovery when clear
                        if (too_close_count[i] < 0) too_close_count[i] = 0;
                    }
                } else {
                    // Downward sensors (5–6): must stay near expected floor distance
                    if (distance_cm < (EXPECTED_DOWNWARD - DOWNWARD_TOLERANCE) ||
                        distance_cm > (EXPECTED_DOWNWARD + DOWNWARD_TOLERANCE)) {
                        downward_fault = true;
                     //   ESP_LOGW(TAG_ECHO, "Downward sensor %d out of range: %.2f cm", i + 1, distance_cm);
                    } else {
                        downward_fault = false;
                    }
                }
            } else {
      //          ESP_LOGW(TAG_ECHO, "Sensor %d OUT OF RANGE or no echo", i + 1);
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Aggregate faults
        sensor_fault = downward_fault ? 1 : 0;
        for (int i = 0; i < 4; i++) {
            if (too_close_count[i] >= FAULT_THRESHOLD) {
                sensor_fault = 1;
                break;
            }
        }

        if (sensor_fault) {
   //         ESP_LOGE(TAG_ECHO, "SENSOR FAULT TRIGGERED");
        } else {
    //        ESP_LOGD(TAG_ECHO, "No sensor fault");
        }

        ESP_LOGD(TAG_ECHO,
                 "Fault counters [%d, %d, %d, %d], Downward=%d",
                 too_close_count[0], too_close_count[1],
                 too_close_count[2], too_close_count[3],
                 downward_fault);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
