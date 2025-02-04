#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#define MAX_ECHO_TIMEOUT_US 40000 // 40ms timeout (~6.8m max range)
#define MEASUREMENT_CYCLE_MS 100   // Suggested delay between measurements

#include <stdio.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include <esp_err.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define GPIO pins for transistors and switches
#define TRANS1 4  // GPIO pin for transistor 1
#define TRANS2 5  // GPIO pin for transistor 2
#define TRANS3 6  // GPIO pin for transistor 3
#define TRANS4 7  // GPIO pin for transistor 4
#define SWITCH1 8 // GPIO pin for switch 1
#define SWITCH2 9 // GPIO pin for switch 2
#define TRIGGER1 10 //GPIO pin for sensor trigger
#define ECHO1 1 //GPIO pin for echo trigger

// Function prototypes
void setup_gpio(void);
void motor_controller(void);
void trigger_sensor(float *distance);
void echo_sensor(void *pvParameter);

static const char *TAG = "ULTRASONIC_SENSOR";

#endif // MOTOR_CONTROL_H