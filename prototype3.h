#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#define MAX_ECHO_TIMEOUT_US 40000 // 40ms timeout (~6.8m max range)
#define MEASUREMENT_CYCLE_MS 100   // Suggested delay between measurements

// PWM configuration
#define PWM_FREQUENCY           16000                   // 5 kHz PWM frequency
#define PWM_DUTY_RESOLUTION     LEDC_TIMER_10_BIT      // 13-bit resolution
#define PWM_DUTY_MAX            ((1 << 10) - 1)        // Maximum duty value (8191 for 13-bit)

// Define PWM channels for the NPN transistors (low-side control)
#define PWM_CHANNEL_FORWARD     LEDC_CHANNEL_0   // Used for TRANS4 (forward)
#define PWM_CHANNEL_REVERSE     LEDC_CHANNEL_1   // Used for TRANS3 (reverse)
#define PWM_TIMER               LEDC_TIMER_0
#define PWM_MODE                LEDC_LOW_SPEED_MODE

// Example speed (50% duty cycle)
#define MOTOR_SPEED_DUTY        (PWM_DUTY_MAX)


#include <stdio.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include <esp_err.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/ledc.h"

// Define GPIO pins for transistors and switches
//H-Bridge 1 Control 
#define TRANS1 19  // GPIO pin for transistor 1
#define TRANS2 20  // GPIO pin for transistor 2
#define TRANS3 21  // GPIO pin for transistor 3
#define TRANS4 47  // GPIO pin for transistor 4
#define SWITCH1 14 // GPIO pin for switch 1
#define SWITCH2 13 // GPIO pin for switch 2
//H-Bridge 2 Control
#define TRANS5 10  // GPIO pin for transistor 1
#define TRANS6 9  // GPIO pin for transistor 2
#define TRANS7 46  // GPIO pin for transistor 3
#define TRANS8 3  // GPIO pin for transistor 4
#define SWITCH3 48 // GPIO pin for switch 1
#define SWITCH4 45 // GPIO pin for switch 2
//Sensor control
#define TRIGGER1 0 //GPIO pin for sensor trigger 1, 3, 5, and 6
#define TRIGGER2 35 //GPIO pin for sensor trigger 2, 4, and 5
#define ECHO1 36 //GPIO pin for echo pin on sensor 1
#define ECHO2 37 //GPIO pin for echo pin on sensor 2
#define ECHO3 38 //GPIO pin for echo pin on sensor 3
#define ECHO4 39 //GPIO pin for echo pin on sensor 4
#define ECHO5 40 //GPIO pin for echo pin on sensor 5
#define ECHO6 41 //GPIO pin for echo pin on sensor 6

// Function prototypes
void setup_gpio(void);
void motor_controller(void);
void trigger_sensor(float *distance);
void echo_sensor(void *pvParameter);
void pwm_init(void);

static const char *TAG = "ULTRASONIC_SENSOR";

#endif // MOTOR_CONTROL_H