/* *****************************************************************************
 * Edge Impulse + INMP441 live inference -> Motor control (ESP32-S3)
 * - Latched voice commands with 1s brake pause between different commands
 * - LEDC PWM on 8 transistor outputs (H-bridge pairs)
 * (c) 2025 CAPSTONE — MIT License
 * *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/ledc.h"                // [ADDED] PWM
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "prototype4.h"

// Edge Impulse
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// ------------------ I2S & audio ------------------
#define I2S_PORT               I2S_NUM_0
#define I2S_SAMPLE_RATE        16000
#define AUDIO_BUFFER_SIZE      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];
static float   features_buffer[AUDIO_BUFFER_SIZE];

// ------------------ Motor pins (EDIT THESE) ------------------
// Avoid I2S pins: BCLK=5, WS=4, SD=6 (note: your earlier comment had 6/5/4 swapped)
/*
#define TRANS1 GPIO_NUM_7
#define TRANS2 GPIO_NUM_8
#define TRANS3 GPIO_NUM_9
#define TRANS4 GPIO_NUM_10
#define TRANS5 GPIO_NUM_11
#define TRANS6 GPIO_NUM_12
#define TRANS7 GPIO_NUM_13
#define TRANS8 GPIO_NUM_14
*/

static const gpio_num_t kAllTrans[] = {
    TRANS1, TRANS2, TRANS3, TRANS4, TRANS5, TRANS6, TRANS7, TRANS8
};

// ------------------ PWM config (NEW) ------------------
#define PWM_FREQ_HZ         16000
#define PWM_RES_BITS        12
#define PWM_RES_ENUM        LEDC_TIMER_12_BIT   // <-- enum for IDF 4.x
#define PWM_MAX_DUTY        ((1U << PWM_RES_BITS) - 1)
#define SPEED_DUTY_PCT      60
#define PWM_BRAKE_PAUSE_MS  1000

// Map one LEDC channel per transistor pin (8 total)
static ledc_channel_config_t s_ch[8];

// ------------------ Command logic ------------------
typedef enum { CMD_NONE=0, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP } Command;

// ---- Per-class thresholds ----
static const float CONF_THRESH_MOTION = 0.95f;  // Forward/Backward/Left/Right
static const float CONF_THRESH_STOP   = 0.30f;  // Stop can be more sensitive (tune)

static const uint8_t STABLE_FRAMES = 1;

// ---------- LATCH + hold knobs ----------
#define LATCH_UNTIL_NEW_COMMAND  1            // [ADDED] keep motion through silence
#define MAX_RUN_MS               25000            // [ADDED] 0 = disabled safety cap
#define MIN_HOLD_MS              10000        // [ADDED] minimum on-time before we would consider auto-stop (we don’t auto-stop)

// ------------------ EI helpers ------------------
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features_buffer + offset, length * sizeof(float));
    return 0;
}

// ------------------ I2S init ------------------
static esp_err_t i2s_init_audio(void) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
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
        .bck_io_num   = GPIO_NUM_5,
        .ws_io_num    = GPIO_NUM_4,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = GPIO_NUM_6
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        ei_printf("I2S install failed: %d\r\n", (int)err);
        return err;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        ei_printf("I2S pin set failed: %d\r\n", (int)err);
        return err;
    }

    return ESP_OK;
}

/*
    ESP_RETURN_ON_ERROR(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL), "I2S", "install fail");
    ESP_RETURN_ON_ERROR(i2s_set_pin(I2S_PORT, &pin_config), "I2S", "pin set fail");
    return ESP_OK;
*/

// ------------------ PWM helpers (NEW) ------------------
static void pwm_init(void) {
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_ENUM,        // <-- enum, not 12
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
    #ifdef LEDC_AUTO_CLK
        .clk_cfg         = LEDC_AUTO_CLK,
    #else
        .clk_cfg         = LEDC_USE_APB_CLK,
    #endif
    };
    ledc_timer_config(&t);

    for (int i = 0; i < 8; i++) {
        s_ch[i] = (ledc_channel_config_t){
            .gpio_num   = kAllTrans[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)(LEDC_CHANNEL_0 + i),
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&s_ch[i]);
    }
}


static inline void pwm_set_duty_idx(int idx, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_ch[idx].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_ch[idx].channel);
}

static void pwm_all_off(void) {
    for (int i = 0; i < 8; i++) pwm_set_duty_idx(i, 0);
}

static uint32_t duty_from_pct(int pct) {
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return (uint32_t)((PWM_MAX_DUTY * (uint64_t)pct) / 100ULL);
}

// ------------------ Patterned drive with PWM (NEW) ------------------
/*
 * Pairing recap (energize pairs with same PWM duty):
 * Left  forward  -> TRANS1 + TRANS4
 * Left  backward -> TRANS2 + TRANS3
 * Right forward  -> TRANS5 + TRANS8
 * Right backward -> TRANS6 + TRANS7
 *
 * Command mapping:
 * Forward  -> Left FORWARD  + Right FORWARD
 * Backward -> Left BACKWARD + Right BACKWARD
 * Left     -> Left BACKWARD + Right FORWARD  (pivot left)
 * Right    -> Left FORWARD  + Right BACKWARD (pivot right)
 * Stop     -> all off
 */
static void drive_pwm(Command cmd, uint32_t duty) {              // [ADDED]
    // Always de-energize first to avoid shoot-through
    pwm_all_off();
    esp_rom_delay_us(400);  // small deadtime

    switch (cmd) {
        case CMD_FORWARD:
            // M1 forward
            pwm_set_duty_idx(0, duty); // T1
            pwm_set_duty_idx(3, duty); // T4
            // M2 forward
            pwm_set_duty_idx(5, duty); // T6
            pwm_set_duty_idx(6, duty); // T7
            break;

        case CMD_BACKWARD:
            // M1 BWD
            pwm_set_duty_idx(1, duty); // T2
            pwm_set_duty_idx(2, duty); // T3
            // M2 BWD
            pwm_set_duty_idx(4, duty); // T5
            pwm_set_duty_idx(7, duty); // T8
            break;

        case CMD_LEFT:
            // M1 BWD 
            pwm_set_duty_idx(1, duty); // T2
            pwm_set_duty_idx(2, duty); // T3
            // M2 FWD
            pwm_set_duty_idx(5, duty); // T6
            pwm_set_duty_idx(6, duty); // T7
            break;

        case CMD_RIGHT:
            // M1 FWD 
            pwm_set_duty_idx(0, duty); // T1
            pwm_set_duty_idx(3, duty); // T4
            // M2 BWD
            pwm_set_duty_idx(4, duty); // T5
            pwm_set_duty_idx(7, duty); // T8
            break;

        case CMD_STOP:
        case CMD_NONE:
        default:
            // already off
            break;
    }
}

// ------------------ Optional: debug print ------------------
static void print_inference_result(const ei_impulse_result_t* result) {
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
              result->timing.dsp, result->timing.classification, result->timing.anomaly);
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: %.5f\r\n",
                  ei_classifier_inferencing_categories[i],
                  result->classification[i].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly: %.3f\r\n", result->anomaly);
#endif
}

// Returns the highest-probability class that exceeds its own threshold.
// If none meet their threshold, returns CMD_NONE.
// *out_val (if not NULL) is set to the winning class's probability (or 0).
static Command top_command_from_result(const ei_impulse_result_t* result, float* out_val) {
    float best_v = 0.0f;
    Command best_cmd = CMD_NONE;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        const char* lbl = ei_classifier_inferencing_categories[i];
        float v = result->classification[i].value;

        // Decide which threshold applies
        float thr =
            (strcmp(lbl, "Stop") == 0) ? CONF_THRESH_STOP : CONF_THRESH_MOTION;

        // Only consider classes that clear their own threshold
        if (v >= thr && v > best_v) {
            if      (strcmp(lbl, "Forward")  == 0) best_cmd = CMD_FORWARD;
            else if (strcmp(lbl, "Backward") == 0) best_cmd = CMD_BACKWARD;
            else if (strcmp(lbl, "Left")     == 0) best_cmd = CMD_LEFT;
            else if (strcmp(lbl, "Right")    == 0) best_cmd = CMD_RIGHT;
            else if (strcmp(lbl, "Stop")     == 0) best_cmd = CMD_STOP;
            else best_cmd = CMD_NONE;

            best_v = v;
        }
    }

    if (out_val) *out_val = best_v;
    return best_cmd;
}


// ------------------ app_main ------------------
extern "C" int app_main(void) {
    // Hardware init
    pwm_init();                         // [ADDED] PWM first (configures pins)
    ei_sleep(100);
    if (i2s_init_audio() != ESP_OK) {
        ei_printf("I2S initialization failed\r\n");
        return 1;
    }

    if (AUDIO_BUFFER_SIZE != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("Buffer size mismatch. Expected %d, got %d\r\n",
                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, AUDIO_BUFFER_SIZE);
        return 1;
    }

    ei_impulse_result_t result = {0};
    ei_printf("Edge Impulse live inference -> PWM motor control (latched)\r\n");

    // Latch state
    Command  last_cmd = CMD_NONE;
    uint8_t  stable   = 0;
    TickType_t last_change_tick = 0;
    const uint32_t duty = duty_from_pct(SPEED_DUTY_PCT);   // [ADDED]

    while (true) {
        // ---- Capture ----
        size_t bytes_read = 0;
        if (i2s_read(I2S_PORT, (void*)audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY) != ESP_OK
            || bytes_read < sizeof(audio_buffer)) {
            ei_printf("Audio capture error: bytes_read=%u\r\n", (unsigned)bytes_read);
            continue;
        }
        // ---- Normalize ----
        for (size_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
            features_buffer[i] = (float)audio_buffer[i] / 32768.0f;
        }
        // ---- Inference ----
        signal_t sig; sig.total_length = AUDIO_BUFFER_SIZE; sig.get_data = raw_feature_get_data;
        EI_IMPULSE_ERROR e = run_classifier(&sig, &result, false);
        if (e != EI_IMPULSE_OK) { ei_printf("ERR: classifier (%d)\r\n", e); return e; }

        // ---- Decide (LATCH + 1s brake on change) ----
        float conf = 0.f;
        Command now = top_command_from_result(&result, &conf);

#if LATCH_UNTIL_NEW_COMMAND
        if (now != CMD_NONE) {
            // update stability only for named commands
            stable = (now == last_cmd) ? (uint8_t)((stable < 255) ? (stable + 1) : 255) : 1;
        }

        // Explicit STOP always wins
        if (now == CMD_STOP && stable >= STABLE_FRAMES) {
            drive_pwm(CMD_STOP, 0);
            last_cmd = CMD_NONE;
            stable = 0;
            last_change_tick = xTaskGetTickCount();
            ei_printf(">> STOP (conf=%.2f)\r\n", conf);
        }
        // New motion command different from current latched motion
        else if (now != CMD_NONE && now != last_cmd && stable >= STABLE_FRAMES) {
            // Brake/stop for a moment
            drive_pwm(CMD_STOP, 0);
            vTaskDelay(pdMS_TO_TICKS(PWM_BRAKE_PAUSE_MS));      // [ADDED] ~1s pause
            // Start new command with PWM
            drive_pwm(now, duty);
            last_cmd = now;
            stable = STABLE_FRAMES;
            last_change_tick = xTaskGetTickCount();
            ei_printf(">> NEW CMD %d (conf=%.2f)\r\n", (int)now, conf);
        }
        // else: keep current PWM running through silence and non-stable blips
#else
        // (Legacy behavior removed for brevity)
#endif

        // ---- Safety (optional) ----
#if (MAX_RUN_MS > 0)
        if (last_cmd != CMD_NONE) {
            TickType_t nowT = xTaskGetTickCount();
            if ((nowT - last_change_tick) > pdMS_TO_TICKS(MAX_RUN_MS)) {
                drive_pwm(CMD_STOP, 0);
                last_cmd = CMD_NONE;
                stable = 0;
                last_change_tick = nowT;
                ei_printf(">> SAFETY TIMEOUT: STOP after %d ms\r\n", (int)MAX_RUN_MS);
            }
        }
#endif

        // Minimum hold timer note:
        // We do NOT auto-stop after MIN_HOLD_MS; we simply keep running until a new command arrives.
        // If you want an auto-stop after 10s of silence instead, set MAX_RUN_MS=10000 and handle CMD_NONE here.
        ei_sleep(20); // small yield
    }
}
