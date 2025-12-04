
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"

// I2S configuration
#define I2S_NUM             I2S_NUM_0
#define SAMPLE_RATE         16000      // Desired audio sample rate
#define I2S_READ_LEN        1024       // Number of samples to read per call

// ESP32-S3 pin assignments for the INMP441 microphone
// Wiring:
//   INMP441 SCK  -> ESP32-S3 GPIO7  (I2S Bit Clock)
//   INMP441 WS   -> ESP32-S3 GPIO8  (I2S Word Select)
//   INMP441 SD   -> ESP32-S3 GPIO9  (I2S Data In)
//   INMP441 L/R  -> Tie to GND for left channel output
#define I2S_BCK_IO          6    // Connect to INMP441 SCK
#define I2S_WS_IO           5    // Connect to INMP441 WS
#define I2S_DATA_IN_IO      4    // Connect to INMP441 SD

static void i2s_configure(void)
{
    // I2S driver configuration for RX mode
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,   // Master mode with RX enabled
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono input from left channel
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,                   // Default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // I2S pin configuration using defined GPIOs
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = -1,    // Not used in RX mode
        .data_in_num = I2S_DATA_IN_IO
    };

    // Install and start the I2S driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

void app_main(void)
{
    // Initialize I2S for the INMP441 microphone
    i2s_configure();

    // Buffer to hold the audio samples
    int16_t i2s_read_buff[I2S_READ_LEN];
    size_t bytes_read;

    printf("Starting audio capture from INMP441 microphone...\n");

    while (1) {
        // Read audio data from I2S
        i2s_read(I2S_NUM, (void*)i2s_read_buff, I2S_READ_LEN * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        // For demonstration: print the first few samples to the serial console
        //printf("Audio samples: ");
        for (int i = 0; i < 10 && i < (bytes_read / sizeof(int16_t)); i++) {
          printf("%d\n", i2s_read_buff[i]);
        }
        //printf("\n");

        //for (int i = 0; i < bytes_read / sizeof(int16_t); i++) {
           // printf("%d\n", i2s_read_buff[i]);
        //}
        // Delay before reading the next batch
        //vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


/*
// Sends bytes to monitor window

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/uart.h"
#include "esp_system.h"

// I2S configuration
#define I2S_NUM             I2S_NUM_0
#define SAMPLE_RATE         16000      // 16 kHz sample rate
#define I2S_READ_LEN        1024       // Number of samples per read

// ESP32-S3 pin assignments for the INMP441 microphone
// Wiring:
//   INMP441 SCK  -> ESP32-S3 GPIO6  (I2S Bit Clock)
//   INMP441 WS   -> ESP32-S3 GPIO5  (I2S Word Select)
//   INMP441 SD   -> ESP32-S3 GPIO4  (I2S Data In)
//   INMP441 L/R  -> Tie to GND for left channel output
#define I2S_BCK_IO          6    // INMP441 SCK
#define I2S_WS_IO           5    // INMP441 WS
#define I2S_DATA_IN_IO      4    // INMP441 SD

// UART configuration for data forwarding
#define UART_NUM            UART_NUM_0
#define UART_BAUD_RATE      2000000    // 2 Mbps high baud rate for streaming

// I2S initialization
static void i2s_configure(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,   // Master mode, receive only
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Single channel (left)
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = -1,    // Not used (RX mode only)
        .data_in_num = I2S_DATA_IN_IO
    };

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

// UART initialization
static void uart_configure(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver with a buffer large enough for streaming
    uart_driver_install(UART_NUM, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_baudrate(UART_NUM, UART_BAUD_RATE);
}

void app_main(void)
{
    // Initialize I2S and UART for high-speed data streaming
    i2s_configure();
    uart_configure();

    int16_t i2s_read_buff[I2S_READ_LEN];
    size_t bytes_read;

    // Continuously read I2S samples and send them over UART
    while (1) {
        // Read a block of samples from I2S
        i2s_read(I2S_NUM, (void*)i2s_read_buff, I2S_READ_LEN * sizeof(int16_t), &bytes_read, portMAX_DELAY);
        
        // Write the raw binary data to UART
        uart_write_bytes(UART_NUM, (const char*)i2s_read_buff, bytes_read);
        
        // No delay is added to maintain continuous streaming at 16kHz
    }
}
*/
