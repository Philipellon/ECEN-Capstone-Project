#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdio.h>
#include "driver/adc.h"



// Define Transistor GPIO Pins (Motor Drivers)
#define TRANS1 21  // Left Motor Reverse
#define TRANS2 47  // Left Motor Forward
#define TRANS3 48  // Left Motor Forward
#define TRANS4 14  // Left Motor Reverse

#define TRANS5 13  // Right Motor Reverse
#define TRANS6 12  // Right Motor Forward
#define TRANS7 2   // Right Motor Forward
#define TRANS8 1   // Right Motor Reverse

// Joystick Input Pins (Analog)
#define JOYSTICK_UD ADC1_6  // Up/Down -> adc pin 5
#define JOYSTICK_LR ADC1_7 // Left/Right -> adc pin 6