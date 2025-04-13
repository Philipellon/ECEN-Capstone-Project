/* Edge Impulse Espressif ESP32 Standalone Live Inference Example using Built-in MEMS Mic
 *
 * This example uses the built-in digital microphone on the Seeed XIAO ESP32S3 Sense,
 * which is internally connected to GPIO8 (WS), GPIO9 (BCLK), and GPIO10 (DATA).
 *
 * It captures live audio via I2S, normalizes the PCM data to floats in the range [-1, 1],
 * and then runs inference with your Edge Impulse model.
 *
 * (c) 2022 EdgeImpulse Inc. - MIT License
 */

/* Standard includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ESP-IDF includes */
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Edge Impulse classifier header */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

#define LED_PIN                GPIO_NUM_21
#define I2S_PORT               I2S_NUM_0
#define I2S_SAMPLE_RATE        16000
#define AUDIO_BUFFER_SIZE      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE   // Must equal the DSP input frame size

// Global buffers: one for raw PCM data and one for normalized float features.
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];
static float features_buffer[AUDIO_BUFFER_SIZE];

/* ------------------------------------------------------------------------- */
/* LED Setup                                                                 */
/* ------------------------------------------------------------------------- */
void setup_led() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_rom_gpio_pad_select_gpio(LED_PIN);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    gpio_pad_select_gpio(LED_PIN);
#endif
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

/* ------------------------------------------------------------------------- */
/* Data callback for the classifier: copies normalized features from buffer */
/* ------------------------------------------------------------------------- */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features_buffer + offset, length * sizeof(float));
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Print inference results                                                   */
/* ------------------------------------------------------------------------- */
void print_inference_result(ei_impulse_result_t result) {
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
              result.timing.dsp,
              result.timing.classification,
              result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: %.5f\r\n",
                  ei_classifier_inferencing_categories[i],
                  result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
}

/* ------------------------------------------------------------------------- */
/* I2S Initialization: Set up I2S driver to capture 16-bit, mono audio at 16kHz  */
/* ------------------------------------------------------------------------- */
esp_err_t i2s_init_audio(void) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),  // Cast here fixes conversion error
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_NUM_6,   // BCLK: GPIO9
        .ws_io_num = GPIO_NUM_5,    // WS: GPIO8
        .data_out_num = I2S_PIN_NO_CHANGE,  // Not used for RX mode
        .data_in_num = GPIO_NUM_4  // Data in: GPIO10 (internal mic)
    };

    esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ei_printf("Failed to install I2S driver\n");
        return ret;
    }
    ret = i2s_set_pin(I2S_PORT, &pin_config);
    if (ret != ESP_OK) {
        ei_printf("Failed to set I2S pins\n");
        return ret;
    }
    return ESP_OK;
}


/* ------------------------------------------------------------------------- */
/* Main Application Entry                                                  */
/* ------------------------------------------------------------------------- */
extern "C" int app_main(void)
{
    setup_led();
    ei_sleep(100);

    // Initialize the I2S driver for live audio capture
    if (i2s_init_audio() != ESP_OK) {
        ei_printf("I2S initialization failed\n");
        return 1;
    }

    ei_impulse_result_t result = { 0 };
    ei_printf("Edge Impulse live inference (ESP32S3) with built-in I2S audio capture\n");

    // Check that the audio buffer size matches the DSP frame size expected by the classifier
    if (AUDIO_BUFFER_SIZE != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("Buffer size mismatch. Expected %d, got %d\n",
                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, AUDIO_BUFFER_SIZE);
        return 1;
    }

    while (true) {
        // Turn LED on to indicate acquisition/inference cycle start
        gpio_set_level(LED_PIN, 1);

        size_t bytes_read = 0;
        esp_err_t ret = i2s_read(I2S_PORT, (void *)audio_buffer,
                                 sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK || bytes_read < sizeof(audio_buffer)) {
            ei_printf("Audio capture error: bytes_read = %u\n", (unsigned)bytes_read);
            continue;
        }

        // Convert raw PCM (int16_t) data to normalized float values
        for (size_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
            features_buffer[i] = (float)audio_buffer[i] / 32768.0f;
        }

        // Create a signal for the Edge Impulse classifier
        signal_t features_signal;
        features_signal.total_length = AUDIO_BUFFER_SIZE;
        features_signal.get_data = raw_feature_get_data;

        // Run inference on the live audio data
        EI_IMPULSE_ERROR infer_err = run_classifier(&features_signal, &result, false /* debug */);
        if (infer_err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", infer_err);
            return infer_err;
        }

        // Print inference results to the serial output
        print_inference_result(result);

        // Turn LED off and wait before next cycle
        gpio_set_level(LED_PIN, 0);
        ei_sleep(1000);
    }
}
