#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"


#define ADC_MAX_VALUE 4095      // For 12-bit ADC resolution
#define VREF 1100               // Default VREF for ESP32, in mV (can vary depending on board)
static const char *TAG = "SENSOR";
#define MAX_ECHO_TIMEOUT_US 100 

// Define Transistor GPIO Pins (Motor Drivers)
#define TRANS1 21  // Left Motor Reverse
#define TRANS2 47  // Left Motor Forward
#define TRANS3 48  // Left Motor Forward
#define TRANS4 14  // Left Motor Reverse

#define TRANS5 13  // Right Motor Reverse
#define TRANS6 12  // Right Motor Forward
#define TRANS7 2   // Right Motor Forward
#define TRANS8 1   // Right Motor Reverse
//Define Sensor Pins
#define ECHO1 7
#define ECHO2 15
#define ECHO3 16
#define ECHO4 17
#define ECHO5 18
#define ECHO6 8
#define TRIGGER1 9
#define TRIGGER2 10
#define TRIGGER3 11

// Joystick Input Pins (Analog)
#define JOYSTICK_UD ADC_CHANNEL_6  // Up/Down -> adc pin 5
#define JOYSTICK_LR ADC_CHANNEL_7 // Left/Right -> adc pin 6

// ADC Calibration
#define DEFAULT_VREF 1100  // Default reference voltage (mV)
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12

// Joystick Calibration
extern int UPMID;   // Center position for Up/Down
extern int LRMID;   // Center position for Left/Right
extern int sensor_fault;


void echo_sensor(void *pvParameter); //1
void motor_controller(void); //2
void init_gpio(void); 
void init_adc(void);
int read_joystick(adc_channel_t channel);
int calibrate_joystick(adc_channel_t channel);
void trigger_sensor(int trigger_pin, int echo_pin, float *distance);

