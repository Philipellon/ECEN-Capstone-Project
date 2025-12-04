// prototype4.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdio.h>
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"

#define SET_POINT       20
#define ADC_MAX_VALUE   4095
#define VREF            1100

// Log tags (macros so they don't collide across TUs)
#define TAG_SENSOR     "SENSOR"
#define TAG_JOYSTICK   "Joystick"
// Back-compat with existing C files that log with TAG1:
#define TAG1           TAG_JOYSTICK

#define MAX_ECHO_TIMEOUT_US 100

// ---------- Transistor pins (typed as gpio_num_t enum) ----------
#define TRANS1  GPIO_NUM_14
#define TRANS2  GPIO_NUM_21
#define TRANS3  GPIO_NUM_47
#define TRANS4  GPIO_NUM_48
#define TRANS5  GPIO_NUM_13
#define TRANS6  GPIO_NUM_12
#define TRANS7  GPIO_NUM_2
#define TRANS8  GPIO_NUM_1

// ---------- Ultrasonic pins ----------
#define ECHO1     GPIO_NUM_7
#define ECHO2     GPIO_NUM_38
#define ECHO3     GPIO_NUM_17
#define ECHO4     GPIO_NUM_18
#define ECHO5     GPIO_NUM_8
#define ECHO6     GPIO_NUM_35

#define TRIGGER1  GPIO_NUM_16
#define TRIGGER2  GPIO_NUM_15
#define TRIGGER3  GPIO_NUM_9
#define TRIGGER4  GPIO_NUM_10
#define TRIGGER5  GPIO_NUM_11
#define TRIGGER6  GPIO_NUM_36

// ---------- Joystick ADC channels ----------
#define JOYSTICK_UD  ADC1_CHANNEL_5
#define JOYSTICK_LR  ADC1_CHANNEL_4

#define DEFAULT_VREF 1100
#define ADC_WIDTH    ADC_WIDTH_BIT_12
#define ADC_ATTEN    ADC_ATTEN_DB_12

// Joystick calibration
extern int UPMID;
extern int LRMID;
extern int sensor_fault;

// Prototypes
void  echo_sensor(void *pvParameter);
void  motor_controller(void);
void  init_gpio(void);
void  init_adc(void);
int   read_joystick(adc_channel_t channel);
int   calibrate_joystick(adc_channel_t channel);
float trigger_sensor(gpio_num_t trigger_pin, gpio_num_t echo_pin);

#ifdef __cplusplus
}
#endif
