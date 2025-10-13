/* *****************************************************************************
 * Edge Impulse + INMP441 live inference -> Motor control (ESP32-S3)
 * - Swap the LED demo for 8 transistor outputs (H-bridge pairs).
 * - Adds confidence threshold + simple 2-frame smoothing.
 * (c) 2025 CAPSTONE — MIT License
 * *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "prototype4.h"

// Edge Impulse
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"      // EI classifier interface (run_classifier, structs)

// ------------------ I2S & audio ------------------
#define I2S_PORT               I2S_NUM_0
#define I2S_SAMPLE_RATE        16000        // 16kHz sampling for voice models
#define AUDIO_BUFFER_SIZE      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE   // Match EI model's frame size requirement
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];         // Raw PCM samples from I2S (signed 16-bit)
static float   features_buffer[AUDIO_BUFFER_SIZE];      // Normalized float features in [-1, 1] for EI

// ------------------ Motor pins (EDIT THESE) ------------------
// Avoid I2S pins: BCLK=6, WS=5, SD=4
/*
#define TRANS1 GPIO_NUM_7   // Left  motor forward A
#define TRANS2 GPIO_NUM_8   // Left  motor backward A
#define TRANS3 GPIO_NUM_9   // Left  motor backward B
#define TRANS4 GPIO_NUM_10  // Left  motor forward B
#define TRANS5 GPIO_NUM_11  // Right motor forward A
#define TRANS6 GPIO_NUM_12  // Right motor backward A
#define TRANS7 GPIO_NUM_13  // Right motor backward B
#define TRANS8 GPIO_NUM_14  // Right motor forward B
*/

static const gpio_num_t kAllTrans[] = {     // Array for all motor transitor pins for loops
    TRANS1, TRANS2, TRANS3, TRANS4, TRANS5, TRANS6, TRANS7, TRANS8  // Pin list
};  // End pin array

// ------------------ Command logic ------------------
typedef enum {      // Command enumeration for motion states
    CMD_NONE = 0, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP  // Possible commands
} Command;      // Alias name ofr the enum type

static const float   CONF_THRESH   = 0.95f;  // tune per model
static const uint8_t STABLE_FRAMES = 1;      // require N identical frames; Smoothing: consecutive agreement count

// ---------- LATCH + optional safety timeout knobs ----------
#define LATCH_UNTIL_NEW_COMMAND  1           // [ADDED] 1 = ignore CMD_NONE; only change on new explicit command
#define MAX_RUN_MS               1500        // [ADDED] safety cap; auto-stop after this many ms (set 0 to disable)

// ------------------ Helpers ------------------
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {        // EI callback to copy feature slices
    memcpy(out_ptr, features_buffer + offset, length * sizeof(float));          // Copy from local feature buffer to EI
    return 0;                                                                   // 0 indicates success to EI SDK
}                                                                               // End raw_feature_get_data

esp_err_t i2s_init_audio(void) {                                // Configure and start I2S for INMP441 microphone
    i2s_config_t i2s_config = {                                 // I2S driver configuration structure
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),    // Master receive mode (we are the clock source)
        .sample_rate = I2S_SAMPLE_RATE,                         // 16kHz sample rate setting
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,           // Capture 16-bit PCM samples
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,            // Mono: use only left channel from I2S stream
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,        // Standard I2S, MSB-first
        .intr_alloc_flags = 0,                                  // Default interrupt allocation
        .dma_buf_count = 4,                                     // Number of DMA buffers
        .dma_buf_len = 256,                                     // Samples of DMA buffer
        .use_apll = false,                                      // Do not use APLL (Standard PLL is sufficient)
        .tx_desc_auto_clear = false,                            // TX not used; keep default
        .fixed_mclk = 0                                         // Auto-compute master clock if required
    };
    i2s_pin_config_t pin_config = {         // Physical pin mapping for I2S signals
        .bck_io_num   = GPIO_NUM_6,         // Bit clock pin (SCK)
        .ws_io_num    = GPIO_NUM_5,         // Word select (WS) pin
        .data_out_num = I2S_PIN_NO_CHANGE,  // No TX data pin (we only receive)
        .data_in_num  = GPIO_NUM_4          // MIC data pin (SD)
    };
    esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);             // Install I2S driver without event queue
    if (ret != ESP_OK) { ei_printf("Failed to install I2S driver\n"); return ret; } // Abort if driver install fails
    ret = i2s_set_pin(I2S_PORT, &pin_config);                                       // Apply I2S pin configuration
    if (ret != ESP_OK) { ei_printf("Failed to set I2S pins\n"); return ret; }       // Abort if pin config fails
    return ESP_OK;
}

// ------------------ Motor control ------------------
static inline void gpio_out(gpio_num_t pin) {       // Helper; prepare a GPIO pin as push-pull output, default LOW
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)   // For ESP-IDF v5+ use new pad select function
    esp_rom_gpio_pad_select_gpio(pin);              // Route pad to GPIO for IDF v5+
#else                                               // Else for older ESP-IDF versions
    gpio_pad_select_gpio(pin);                      // Route pad to GPIO for pre-V5
#endif                                              // End version conditional
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);      // Configure direction to output
    gpio_set_level(pin, 0);                         // Initialize level LOW (safe idle)
}                                                   // End gpio_out

void motors_init(void) {                                                    // Initialize all transistor GPIOs as outputs
    for (size_t i = 0; i < sizeof(kAllTrans)/sizeof(kAllTrans[0]); ++i) {   // iterate over each mapped pin
        gpio_out(kAllTrans[i]);                                             // configure pin as output and set LOW
    }
}

void motors_all_low(void) {                                                 // Set all transistor outputs LOW (safe stop)
    for (size_t i = 0; i < sizeof(kAllTrans)/sizeof(kAllTrans[0]); ++i) {   // Loop through all motor pins
        gpio_set_level(kAllTrans[i], 0);                                    // Drive pin LOW
    }
}

/*
 * Partner mapping (safety: always drop to LOW first):
 * Left  forward  -> TRANS1 + TRANS4 = HIGH
 * Left  backward -> TRANS2 + TRANS3 = HIGH
 * Right forward  -> TRANS5 + TRANS8 = HIGH
 * Right backward -> TRANS6 + TRANS7 = HIGH
 *
 * Commands mapping:
 * Forward  -> Left FORWARD  + Right BACKWARD
 * Backward -> Left BACKWARD + Right FORWARD
 * Left     -> Both BACKWARD (on-spot left turn)
 * Right    -> Both FORWARD  (on-spot right turn)
 * Stop     -> All LOW
 */
void drive(Command cmd) {       // Apply transistor pattern for the requested command
    // interlock: de-energize before changing pattern
    motors_all_low();           // Ensure all outputs LOW to avoid shoot-through
    // tiny deadtime (tune if needed)
    esp_rom_delay_us(500);      // Add a short deadtime (500us) before new pattern

switch (cmd) {                                                      // Select behavior based on command
case CMD_FORWARD:                                                   // Forward: Left FWD + right BWD
            gpio_set_level(TRANS1, 1); gpio_set_level(TRANS4, 1);   // Left FWD pair ON
            gpio_set_level(TRANS6, 1); gpio_set_level(TRANS7, 1);   // Right BWD pair ON
            break;                                                  
        case CMD_BACKWARD:                                          // Backward: left BWD + right FWD
            gpio_set_level(TRANS1, 1); gpio_set_level(TRANS4, 1);   // Left FWD pair ON
            gpio_set_level(TRANS5, 1); gpio_set_level(TRANS8, 1);   // Right FWD pair ON
            break;                                                  
        case CMD_LEFT:                                              // Left spin: both BWD 
            gpio_set_level(TRANS2, 1); gpio_set_level(TRANS3, 1);   // Left BWD pair ON
            gpio_set_level(TRANS6, 1); gpio_set_level(TRANS7, 1);   // Right BWD pair ON
            break;                                                  
        case CMD_RIGHT:                                             // Right spin: both FWD
            gpio_set_level(TRANS2, 1); gpio_set_level(TRANS3, 1);   // Left BWD pair ON
            gpio_set_level(TRANS5, 1); gpio_set_level(TRANS8, 1);   // Right FWD pain ON
            break;                                                  
        case CMD_STOP:                                              // Stop command
        case CMD_NONE:                                              // Or no command 
        default:                                                    // Default safety case
            // already LOW
            break;
    }
}

// Get top label + value; returns CMD_NONE if below threshold
Command top_command_from_result(const ei_impulse_result_t& result, float* out_val) {    // Map EI result to Command
    uint16_t best_i = 0;        // Index of best label seen
    float best_v = 0.0f;        // Highest probability value seen 
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {  // Iterate over all labels 
        float v = result.classification[i].value;               // Read probability for label i
        if (v > best_v) { best_v = v; best_i = i; }             // Track new maximum and its index
    }
    if (out_val) *out_val = best_v;             // Return best probability to caller if requested
    if (best_v < CONF_THRESH) return CMD_NONE;  // Enforce confidence threshold gate

    const char* lbl = ei_classifier_inferencing_categories[best_i];     // Get label text for best index 
    if      (strcmp(lbl, "Forward")  == 0) return CMD_FORWARD;          // Map Forward to enum
    else if (strcmp(lbl, "Backward") == 0) return CMD_BACKWARD;         // Map Backward to enum
    else if (strcmp(lbl, "Left")     == 0) return CMD_LEFT;             // Map Left to enum
    else if (strcmp(lbl, "Right")    == 0) return CMD_RIGHT;            // Map Right to enum
    else if (strcmp(lbl, "Stop")     == 0) return CMD_STOP;             // Map Stop to enum
    return CMD_NONE;
}

// ------------------ Optional: debug print ------------------
void print_inference_result(const ei_impulse_result_t& result) {        // Print timing and predicitons
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",  // Print timing information
              result.timing.dsp, result.timing.classification, result.timing.anomaly);  // Fields from EI result
    ei_printf("Predictions:\r\n");      // Header for label scores
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {  // Loop through each label
        ei_printf("  %s: %.5f\r\n",                     // Print label name and probability (5 decimals)
            ei_classifier_inferencing_categories[i],    // Label text
            result.classification[i].value);            // Probability for that label
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1      // If anomaly score is available in the model
    ei_printf("Anomaly: %.3f\r\n", result.anomaly); // print anomaly score
#endif
}

// ------------------ app_main ------------------
extern "C" int app_main(void) {     // ESP-IDF application entry point (C linkage)
    motors_init();                  // Configure motor GPIOs as outputs and set LOW
    ei_sleep(100);                  // Small delay to let hardware settle

    if (i2s_init_audio() != ESP_OK) {               // Initialize I2S; check for success   
        ei_printf("I2S initialization failed\n");   // Report failure to console
        return 1;                                   
    }

    ei_impulse_result_t result = {0};               // Zero-initialize EI result struct
    ei_printf("Edge Impulse live inference -> motor control (ESP32-S3)\r\n");   // Startup banner

    if (AUDIO_BUFFER_SIZE != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {          // Sanity check: buffer size matches model
        ei_printf("Buffer size mismatch. Expected %d, got %d\n",            // Print expected vs actual
                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, AUDIO_BUFFER_SIZE);   // Values from macros
        return 1;                                                           // Abort if mismatch to avoid misaligned inference
    }

    Command last_cmd = CMD_NONE;        // Track last applied command for smoothing logic
    uint8_t stable = 0;                 // Count of consecutive identical command frames

    TickType_t last_drive_tick = 0;     // [ADDED] track when drive() last changed outputs (for safety timeout)

    while (true) {      // Main loop: capture -> infer -> decide -> drive
        // ---- Capture audio ----
        size_t bytes_read = 0;                                      // Will hold number of bytes read from I2S
        esp_err_t ret = i2s_read(I2S_PORT, (void*)audio_buffer,     // Read one frame from I2S into PCM buffer
                                 sizeof(audio_buffer), &bytes_read, portMAX_DELAY);     // Block until frame is ready
        if (ret != ESP_OK || bytes_read < sizeof(audio_buffer)) {                       // Validate full frame and no error
            ei_printf("Audio capture error: bytes_read = %u\n", (unsigned)bytes_read);  // Log capture issue
            continue;                                                                   // Skip to next loop iteration on capture failure
        }
        // ---- Normalize to [-1,1] ----
        for (size_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {            // Convert each sample to float
            features_buffer[i] = (float)audio_buffer[i] / 32768.0f; // Normalize int16_t -> [-1. 1]
        }
        // ---- Run inference ----
        signal_t sig; sig.total_length = AUDIO_BUFFER_SIZE; sig.get_data = raw_feature_get_data;    // Prepare EI signal
        EI_IMPULSE_ERROR e = run_classifier(&sig, &result, false);      // Run EI inference (no debug prints)
        if (e != EI_IMPULSE_OK) { ei_printf("ERR: classifier (%d)\n", e); return e; }   // Abort on classifier error

        // Optional debug
        print_inference_result(result);     // Print timing and predictions to console

        // ---- Decide + (CHANGED to LATCH) ----
        float conf = 0.f;                                      // Holds best confidence for current frame
        Command now = top_command_from_result(result, &conf);  // Map predictions to Command enum

#if LATCH_UNTIL_NEW_COMMAND                                   // [ADDED] LATCH: ignore CMD_NONE (silence)
        // Update stability only when we have a named command (not NONE)
        if (now != CMD_NONE) {
            stable = (now == last_cmd) ? (uint8_t)((stable < 255) ? (stable + 1) : 255) : 1;
        }

        // Highest priority: explicit STOP (requires stability)
        if (now == CMD_STOP && stable >= STABLE_FRAMES) {
            drive(CMD_STOP);
            last_cmd = CMD_NONE;
            stable = 0;
            last_drive_tick = xTaskGetTickCount();            // [ADDED] update safety timer
            ei_printf(">> LATCH: STOP (conf=%.2f)\r\n", conf);
        }
        // New motion command (requires stability)
        else if (now != CMD_NONE && now != last_cmd && stable >= STABLE_FRAMES) {
            drive(now);
            last_cmd = now;
            stable = STABLE_FRAMES;                           // keep at threshold to avoid spurious bumps
            last_drive_tick = xTaskGetTickCount();            // [ADDED] update safety timer
            ei_printf(">> LATCH: %d (conf=%.2f)\r\n", (int)now, conf);
        }
        // else: now == CMD_NONE -> do nothing; keep last_cmd driving
#else
        // -------- ORIGINAL dead-man behavior (kept for reference) --------
        if (now != CMD_NONE && now == last_cmd) {
            if (stable < 255) stable++;
        } else {
            stable = 1;
        }
        if (now == CMD_NONE) {
            drive(CMD_STOP);
            last_cmd = CMD_NONE;
            stable = 0;
            last_drive_tick = xTaskGetTickCount();            // [ADDED] update safety timer
        } else if (stable >= STABLE_FRAMES && now != last_cmd) {
            drive(now);
            last_cmd = now;
            stable = STABLE_FRAMES;
            last_drive_tick = xTaskGetTickCount();            // [ADDED] update safety timer
            ei_printf(">> DRIVE: %d (conf=%.2f)\r\n", (int)now, conf);
        } else if (last_cmd == CMD_NONE && now != CMD_NONE && stable >= STABLE_FRAMES) {
            drive(now);
            last_cmd = now;
            last_drive_tick = xTaskGetTickCount();            // [ADDED] update safety timer
            ei_printf(">> DRIVE from STOP: %d (conf=%.2f)\r\n", (int)now, conf);
        }
#endif

        // ---- Safety timeout (optional) ----
#if (MAX_RUN_MS > 0)                                          // [ADDED] hard cap run time for safety
        if (last_cmd != CMD_NONE) {
            TickType_t nowTick = xTaskGetTickCount();
            if ((nowTick - last_drive_tick) > pdMS_TO_TICKS(MAX_RUN_MS)) {
                drive(CMD_STOP);
                last_cmd = CMD_NONE;
                stable = 0;
                last_drive_tick = nowTick;
                ei_printf(">> SAFETY TIMEOUT: STOP after %d ms\r\n", (int)MAX_RUN_MS);
            }
        }
#endif

        // NOTE: Your EI window likely ~1.0 s; this sleep does not change latency
        ei_sleep(50); // small yield; keep responsive without impacting capture, short delay to yield CPU
    }
}
