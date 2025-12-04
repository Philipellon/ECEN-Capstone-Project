/* *****************************************************************************
 * Edge Impulse Espressif ESP32 Standalone Live Inference Example using 
 * Built-in MEMS Mic
 *
 * It captures live audio via I2S, normalizes the PCM data to floats in the range [-1, 1],
 * and then runs inference with your Edge Impulse model.
 *
 * (c) 2022 EdgeImpulse Inc. - MIT License
 * *****************************************************************************/

// ------------------------- Standard includes ------------------------------ //
#include <stdio.h>                      // Standard C library for input/output functions
#include <string.h>                     // Standard C library for string manipulation functions
#include <stdlib.h>                     // Standard C library for general utilities (e.g., dynamic memory)
#include "driver/gpio.h"                // ESP-IDF driver for General Purpose Input/Output (GPIO)
#include "driver/i2s.h"                 // ESP-IDF driver for I2S interface for audio communication
#include "sdkconfig.h"                  // SDK configuration settings specific to this project
#include "esp_idf_version.h"            // Library to check version macros of ESP-IDF
#include "freertos/FreeRTOS.h"          // FreeRTOS core header for multitasking support
#include "freertos/task.h"              // FreeRTOS task management functions

// ------------------- Edge Impulse classifier header ----------------------- //
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" // Edge Impulse header for running the classifier

// ----------------- LED Pin Definitions for each command --------------------- //
// Define the GPIO pins used to indicate different commands via LEDs
#define LED_BACKWARD GPIO_NUM_13         // GPIO pin 13 assigned for the "Backward" command LED
#define LED_FORWARD  GPIO_NUM_12         // GPIO pin 12 assigned for the "Forward" command LED
#define LED_LEFT     GPIO_NUM_11         // GPIO pin 11 assigned for the "Left" command LED
#define LED_RIGHT    GPIO_NUM_10         // GPIO pin 10 assigned for the "Right" command LED
#define LED_STOP     GPIO_NUM_9          // GPIO pin 9 assigned for the "Stop" command LED

// ----------------------------- Other Definitions ---------------------------- //
// Define constants for I2S audio configuration and buffer sizes
#define I2S_PORT               I2S_NUM_0            // Use I2S port number 0 for audio capture
#define I2S_SAMPLE_RATE        16000                // Set the audio sample rate to 16kHz (16,000 samples per second)
#define AUDIO_BUFFER_SIZE      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // Define the audio buffer size to match the classifier's DSP input frame size

// -------------------------- Global Buffer Variables ------------------------- //
// Global buffer for raw PCM audio data as 16-bit signed integers
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];
// Global buffer for normalized audio features as floating point values in the range [-1, 1]
static float features_buffer[AUDIO_BUFFER_SIZE];

// ------------------------- Setup Command LEDs Function ---------------------- //
/* 
 * Function: setup_command_leds
 * ----------------------------
 * Initializes all GPIO pins designated for command LEDs by setting each LED pin
 * to be a digital output and ensuring they are turned off initially.
 */
void setup_command_leds() {
    // Create an array of GPIO pin identifiers for all command LEDs.
    gpio_num_t leds[] = {LED_BACKWARD, LED_FORWARD, LED_LEFT, LED_RIGHT, LED_STOP};
    
    // Loop over all LED pins to configure them individually.
    for (int i = 0; i < 5; i++) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        // For ESP-IDF v5.0.0 or later, use the ROM function to select the GPIO pad.
        esp_rom_gpio_pad_select_gpio(leds[i]);
#else
        // For earlier versions of ESP-IDF, use the standard function to select the GPIO pad.
        gpio_pad_select_gpio(leds[i]);
#endif
        // Set the current LED pin direction as an output.
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        // Set the current LED pin level to LOW (turn off the LED).
        gpio_set_level(leds[i], 0);  // Ensure the LED is initially off
    }
}

// --------------------- Raw Feature Data Callback Function ------------------- //
/*
 * Function: raw_feature_get_data
 * ------------------------------
 * Provides normalized feature data to the classifier.
 * It copies a section of the global 'features_buffer' into an output buffer.
 *
 * Parameters:
 *   offset   - The starting index in the features_buffer from where data is copied.
 *   length   - The number of float elements to copy.
 *   out_ptr  - Pointer to the output buffer where the data is to be stored.
 *
 * Returns:
 *   0 on success.
 */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    // Copy 'length' floats starting from the 'offset' position in features_buffer into out_ptr.
    memcpy(out_ptr, features_buffer + offset, length * sizeof(float));
    // Return 0 to indicate success.
    return 0;
}

// ------------------------ Print Inference Results Function ------------------ //
/*
 * Function: print_inference_result
 * --------------------------------
 * Prints the results from the classifier inference to the serial output.
 * It displays timing information and either the object detection bounding boxes 
 * or classification predictions depending on the build configuration.
 *
 * Parameters:
 *   result - The structure containing the classifier's inference results.
 */
void print_inference_result(ei_impulse_result_t result) {
    // Print timing information for DSP processing, classification, and anomaly detection.
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
              result.timing.dsp,
              result.timing.classification,
              result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    // If object detection mode is enabled, print bounding boxes information.
    ei_printf("Object detection bounding boxes:\r\n");
    // Loop through each detected bounding box.
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        // Retrieve the current bounding box from the result.
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        // Skip bounding boxes with a confidence value of 0.
        if (bb.value == 0) continue;
        // Print details for the bounding box: label, confidence, and coordinates.
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
#else
    // If object detection is not enabled, print general classification predictions.
    ei_printf("Predictions:\r\n");
    // Loop through each classification category.
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        // Print the label and corresponding probability for each category.
        ei_printf("  %s: %.5f\r\n",
                  ei_classifier_inferencing_categories[i],
                  result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    // If anomaly detection is enabled, print the anomaly score.
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
}

// ------------------------- I2S Audio Initialization Function ---------------- //
/*
 * Function: i2s_init_audio
 * ------------------------
 * Initializes the I2S driver for capturing 16-bit, mono audio at a sample rate of 16kHz.
 *
 * Returns:
 *   ESP_OK if initialization is successful; an error code otherwise.
 */
esp_err_t i2s_init_audio(void) {
    // Configure the I2S driver with specified mode, sample rate, and buffer properties.
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Set as master and receiver mode.
        .sample_rate = I2S_SAMPLE_RATE,                       // Set the sampling rate to 16kHz.
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,         // Use 16-bit samples.
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // Capture only one channel (mono).
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,      // Use most-significant-bit first format.
        .intr_alloc_flags = 0,                                // Default interrupt allocation flags.
        .dma_buf_count = 4,                                   // Number of DMA buffers.
        .dma_buf_len = 256,                                   // Length of each DMA buffer.
        .use_apll = false,                                    // Do not use the APLL clock.
        .tx_desc_auto_clear = false,                          // Do not auto-clear transmission descriptor.
        .fixed_mclk = 0                                       // No fixed master clock frequency.
    };

    // Set the pin configuration for I2S.
    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_NUM_6,          // BCLK (Bit Clock) assigned to GPIO6.
        .ws_io_num = GPIO_NUM_5,           // WS (Word Select) assigned to GPIO5.
        .data_out_num = I2S_PIN_NO_CHANGE, // Data out not used for RX mode.
        .data_in_num = GPIO_NUM_4          // Data in assigned to GPIO4 for internal mic input.
    };

    // Install the I2S driver with the defined configuration.
    esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    // Check if the driver installation was successful.
    if (ret != ESP_OK) {
        // Print error message if the I2S driver fails to install.
        ei_printf("Failed to install I2S driver\n");
        return ret;  // Return the error code.
    }
    // Configure the I2S pins with the previously defined pin configuration.
    ret = i2s_set_pin(I2S_PORT, &pin_config);
    // Check if the pin configuration was successful.
    if (ret != ESP_OK) {
        // Print error message if setting the I2S pins fails.
        ei_printf("Failed to set I2S pins\n");
        return ret;  // Return the error code.
    }
    // Return success code upon successful initialization.
    return ESP_OK;
}

// ---------------------- Include for String Comparison ----------------------- //
#include <string.h> // Include library for string operations, especially for strcmp used later

// --------------------- Update Command LEDs Function ------------------------- //
/*
 * Function: update_command_leds
 * -----------------------------
 * Updates the state of command LEDs based on classification results.
 * Turns off all LEDs and then activates any LED whose corresponding 
 * inferred confidence is above the threshold (0.5 in this case).
 *
 * Parameters:
 *   result - The structure containing the classifier's inference results.
 */
void update_command_leds(ei_impulse_result_t result) {
    // Turn off all command LEDs before checking the latest inference.
    gpio_set_level(LED_BACKWARD, 0);  // Turn off "Backward" LED.
    gpio_set_level(LED_FORWARD, 0);   // Turn off "Forward" LED.
    gpio_set_level(LED_LEFT, 0);      // Turn off "Left" LED.
    gpio_set_level(LED_RIGHT, 0);     // Turn off "Right" LED.
    gpio_set_level(LED_STOP, 0);      // Turn off "Stop" LED.

    // Loop through each classification label.
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        // Check if the classifier's confidence exceeds the threshold (0.5).
        if (result.classification[i].value > 0.5) {
            // Compare the classification label to "Backward".
            if (strcmp(ei_classifier_inferencing_categories[i], "Backward") == 0) {
                // If it matches "Backward", turn on the corresponding LED.
                gpio_set_level(LED_BACKWARD, 1);
                ei_printf("Backward LED on");
            } 
            // Compare the classification label to "Forward".
            else if (strcmp(ei_classifier_inferencing_categories[i], "Forward") == 0) {
                // If it matches "Forward", turn on the corresponding LED.
                gpio_set_level(LED_FORWARD, 1);
            } 
            // Compare the classification label to "Left".
            else if (strcmp(ei_classifier_inferencing_categories[i], "Left") == 0) {
                // If it matches "Left", turn on the corresponding LED.
                gpio_set_level(LED_LEFT, 1);
            } 
            // Compare the classification label to "Right".
            else if (strcmp(ei_classifier_inferencing_categories[i], "Right") == 0) {
                // If it matches "Right", turn on the corresponding LED.
                gpio_set_level(LED_RIGHT, 1);
            } 
            // Compare the classification label to "Stop".
            else if (strcmp(ei_classifier_inferencing_categories[i], "Stop") == 0) {
                // If it matches "Stop", turn on the corresponding LED.
                gpio_set_level(LED_STOP, 1);
            }
        }
    }
}

// ----------------------- Main Application Entry Point ----------------------- //
/*
 * Function: app_main
 * ------------------
 * The entry point for the ESP32 application. It initializes hardware,
 * continuously captures audio, processes it for inference via the Edge Impulse model,
 * prints the inference results, and updates the command LEDs accordingly.
 */
extern "C" int app_main(void)
{
    // Initialize command LEDs (setup GPIO outputs for each command LED).
    setup_command_leds();
    // Wait for 100ms to allow LEDs to initialize properly.
    ei_sleep(100);

    // Initialize the I2S driver to capture live audio.
    if (i2s_init_audio() != ESP_OK) {
        // If I2S initialization fails, print an error message.
        ei_printf("I2S initialization failed\n");
        // Return error code to indicate failure.
        return 1;
    }

    // Declare and initialize the inference result structure.
    ei_impulse_result_t result = { 0 };
    // Print introductory message indicating the start of live inference.
    ei_printf("Edge Impulse live inference (ESP32S3) with built-in I2S audio capture\n");

    // Verify that the size of the audio buffer matches the expected classifier DSP input frame size.
    if (AUDIO_BUFFER_SIZE != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        // Print an error if the buffer size does not match.
        ei_printf("Buffer size mismatch. Expected %d, got %d\n",
                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, AUDIO_BUFFER_SIZE);
        // Return error code since the sizes must match.
        return 1;
    }

    // Begin an infinite loop to continuously capture audio and run inference.
    while (true) {
        // Variable to store the number of bytes read from the I2S interface.
        size_t bytes_read = 0;
        // Read audio data from the I2S peripheral into the audio_buffer.
        esp_err_t ret = i2s_read(I2S_PORT, (void *)audio_buffer,
                                 sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
        // If there is an error with reading or fewer bytes are read than expected:
        if (ret != ESP_OK || bytes_read < sizeof(audio_buffer)) {
            // Print an error message including the number of bytes actually read.
            ei_printf("Audio capture error: bytes_read = %u\n", (unsigned)bytes_read);
            // Skip the rest of this loop iteration and continue to the next cycle.
            continue;
        }

        // Normalize the raw PCM audio data to floating point values in the range [-1, 1]
        for (size_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
            // Convert each 16-bit sample to a float by dividing by 32768.0f (the max absolute value for 16-bit PCM)
            features_buffer[i] = (float)audio_buffer[i] / 32768.0f;
        }

        // Create a signal structure for the classifier with the normalized audio features.
        signal_t features_signal;
        // Total length of the audio data in the signal.
        features_signal.total_length = AUDIO_BUFFER_SIZE;
        // Set the callback function used by the classifier to obtain audio data.
        features_signal.get_data = raw_feature_get_data;

        // Run inference on the prepared live audio signal using the Edge Impulse classifier.
        EI_IMPULSE_ERROR infer_err = run_classifier(&features_signal, &result, false);
        // Check if the classifier run was successful.
        if (infer_err != EI_IMPULSE_OK) {
            // Print an error message with the error code if inference fails.
            ei_printf("ERR: Failed to run classifier (%d)\n", infer_err);
            // Return the error code and exit.
            return infer_err;
        }

        // Print the inference results (timing, predictions, anomaly scores, etc.) to the serial output.
        print_inference_result(result);

        // Update the command LEDs based on the classification results.
        update_command_leds(result);

        // Wait for 1000ms before starting the next inference cycle.
        // During this period, any activated LEDs remain on if their corresponding command was detected.
        ei_sleep(1000);
    }
}
