#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

// NEW Continuous ADC driver header (ESP-IDF 5.x on ESP32-S3)
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h" // For adc_channel_t, adc_bitwidth_t, etc.

static const char *TAG = "ADC_CONT_EXAMPLE";

// Adjust these for your wiring and desired sample rate
#define EXAMPLE_ADC_CHANNEL     ADC_CHANNEL_4   // e.g. GPIO4 → ADC1_CH4 (check your board’s pin-to-channel map)
#define EXAMPLE_SAMPLE_FREQ_HZ  (16000)         // ~16 kHz
#define EXAMPLE_READ_LEN        512             // how many bytes we read per loop iteration

// We'll store the ADC driver handle globally
static adc_continuous_handle_t s_adc_handle = NULL;

void app_main(void)
{
    esp_err_t ret;

    // 1) Configure the "pattern" for one ADC channel
    //    Each 'adc_digi_pattern_config_t' sets up one channel's bit width, attenuation, etc.
    adc_digi_pattern_config_t adc_pattern[1] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_11;         // 0-3.6V range approx.
    adc_pattern[0].channel = EXAMPLE_ADC_CHANNEL;   // e.g. channel 4 for GPIO4
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;     // ESP32-S3 supports 12-bit
    adc_pattern[0].unit = 0;                        // 0 => ADC_UNIT_1

    // 2) Set overall continuous ADC config
    adc_continuous_config_t adc_config = {
        .pattern_num = 1,                // number of channels/patterns
        .adc_pattern = adc_pattern,      // our single-channel pattern
        .sample_freq_hz = EXAMPLE_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,  // using ADC1 only
        // TYPE2 format = 4 bytes per sample with channel info + raw data
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2
    };

    // 3) Create the ADC driver handle, specifying buffer sizes
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 2048,  // internal driver buffer
        .conv_frame_size = 256,      // driver fetches data in chunks
    };

    ret = adc_continuous_new_handle(&handle_cfg, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC handle: %s", esp_err_to_name(ret));
        return;
    }

    // 4) Apply the ADC configuration
    ret = adc_continuous_config(s_adc_handle, &adc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %s", esp_err_to_name(ret));
        return;
    }

    // 5) Start continuous ADC sampling
    ret = adc_continuous_start(s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "ADC continuous sampling started at %d Hz...", EXAMPLE_SAMPLE_FREQ_HZ);

    // 6) Continuously read blocks of raw ADC data in a loop
    while (1) {
        uint8_t result[EXAMPLE_READ_LEN] = {0};
        uint32_t bytes_read = 0;

        // This call will block (up to 1000 ms) until at least 1 byte is available
        ret = adc_continuous_read(s_adc_handle, result, EXAMPLE_READ_LEN,
                                  &bytes_read, 1000 /* timeout ms */);

        if (ret == ESP_OK && bytes_read > 0) {
            // 'bytes_read' is the number of bytes actually read
            int num_samples = bytes_read / 4; // TYPE2 = 4 bytes per sample
            // We'll just print the first couple of samples to see if they vary
            for (int i = 0; i < num_samples && i < 2; i++) {
                // Each sample is 4 bytes => channel info + raw data
                uint32_t word = (result[4*i + 3] << 24) |
                                (result[4*i + 2] << 16) |
                                (result[4*i + 1] <<  8) |
                                (result[4*i + 0] <<  0);
                // channel is usually in bits [16:13], raw data in bits [12:0] for 13-bit
                uint32_t channel = (word >> 13) & 0xF;
                uint32_t adc_val = word & 0x1FFF; // 13 bits => max ~8191

                ESP_LOGI(TAG, "Sample %d: channel=%lu, raw=%lu",
                         i, (unsigned long)channel, (unsigned long)adc_val);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // If it's timing out, you might not be generating enough samples or
            // your sampling rate is too low relative to how often you're reading.
            ESP_LOGW(TAG, "ADC read timed out");
        } else {
            ESP_LOGE(TAG, "adc_continuous_read error: %s", esp_err_to_name(ret));
        }

        // Delay so we don't spam logs too quickly
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
