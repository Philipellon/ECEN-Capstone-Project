#include "prototype1.h"

void echo_sensor(void *pvParameter){
    float distance;
    while (true) {
        trigger_sensor(&distance);
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_CYCLE_MS)); // Prevent overlapping signals }
    }
}
