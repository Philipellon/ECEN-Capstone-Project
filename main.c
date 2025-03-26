#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_adc/adc_continuous.h"

#define SAMPLE_RATE_HZ    16000
#define ADC_UNIT          ADC_UNIT_1
#define ADC_CHANNEL       ADC_CHANNEL_4    // Adjust for your wiring
#define ADC_ATTEN         ADC_ATTEN_DB_12  // Use ADC_ATTEN_DB_12 (DB_11 is deprecated)
#define DMA_BUF_LEN       1024             // DMA buffer size in bytes

static adc_continuous_handle_t adc_handle = NULL;

// Define the ADC pattern as a static array.
static adc_digi_pattern_config_t adc_pattern[1] = {
    {
        .atten     = ADC_ATTEN,
        .channel   = ADC_CHANNEL,
        .unit      = ADC_UNIT,
        .bit_width = ADC_BITWIDTH_12,  // Use 12-bit resolution
    }
};

void app_main(void)
{
    esp_err_t ret;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size = DMA_BUF_LEN,
    };

    ret = adc_continuous_new_handle(&handle_cfg, &adc_handle);
    if (ret != ESP_OK) {
        printf("Failed to create ADC continuous handle: %d\n", ret);
        return;
    }

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,  // Specify TYPE2 as required
        .pattern_num = 1,
        .adc_pattern = adc_pattern,              // Use the static array defined above
    };

    ret = adc_continuous_config(adc_handle, &adc_config);
    if (ret != ESP_OK) {
        printf("Failed to configure ADC continuous: %d\n", ret);
        return;
    }

    ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) {
        printf("Failed to start ADC continuous: %d\n", ret);
        return;
    }

    uint8_t result[DMA_BUF_LEN];
    uint32_t ret_len = 0;

    while (1) {
        ret = adc_continuous_read(adc_handle, result, DMA_BUF_LEN, &ret_len, 1000);
        if (ret == ESP_OK) {
            // In TYPE2 format, each sample is 4 bytes.
            int samples = ret_len / sizeof(uint32_t);
            uint32_t *data = (uint32_t*) result;
            for (int i = 0; i < samples; i++) {
                adc_digi_output_data_t sample;
                sample.val = data[i];
                printf("Channel: %d, ADC Value: %d\n", sample.type2.channel, sample.type2.data);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // No data available within timeout.
        } else {
            printf("ADC continuous read error: %d\n", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Clean up code (not reached in this infinite loop):
    // adc_continuous_stop(adc_handle);
    // adc_continuous_deinit(adc_handle);
}
