#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"  


#define SET_POINT 20 // Set to Adjust responsiveness of Joystick input increase to decrease responsiveness and decrease to increase responsiveness
#define ADC_MAX_VALUE 4095      // For 12-bit ADC resolution
#define VREF 1100               // Default VREF for ESP32, in mV (can vary depending on board)
static const char *TAG = "SENSOR";
static const char *TAG1 = "Joystick";
#define MAX_ECHO_TIMEOUT_US 100 
extern int sensor_fault;

// Define Transistor GPIO Pins (Motor Drivers)
#define TRANS1 14  // GPIO pin for transistor R1
#define TRANS2 21 // GPIO pin for transistor R2
#define TRANS3 47 // GPIO pin for transistor R7
#define TRANS4 48 // GPIO pin for transistor R8
#define TRANS5 13 // GPIO pin for transistor R9
#define TRANS6 12 // GPIO pin for transistor R10
#define TRANS7 2 // GPIO pin for transistor R15
#define TRANS8 1 // GPIO pin for transistor R16
//Define Sensor Pins
#define ECHO1 7 // checked
#define ECHO2 20 //checked
#define ECHO3 16
#define ECHO4 17
#define ECHO5 18 //right
#define ECHO6 8 //checked
#define TRIGGER1 44 //checked
#define TRIGGER2 15 //checked
#define TRIGGER3 9 
#define TRIGGER4 10
#define TRIGGER5 11 //right
#define TRIGGER6 36 // checked

// Joystick Input Pins (Analog)
#define JOYSTICK_UD ADC1_CHANNEL_5  // Up/Down -> adc pin 6
#define JOYSTICK_LR ADC1_CHANNEL_4 // Left/Right -> adc pin 5

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
float trigger_sensor(int trigger_pin, int echo_pin);

