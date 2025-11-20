#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "driver/ledc.h"
#include "driver/i2s.h"
#include "esp_rom_sys.h"
#include "esp_http_server.h"
#include "prototype4.h"
#include "driver/gpio.h"

// Edge Impulse
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// ---------- Forward decls for sensor task (C files) ----------
extern "C" {
    void  echo_sensor(void *pvParameter);
    float trigger_sensor(gpio_num_t trigger_pin, gpio_num_t echo_pin);
    extern int sensor_fault;  // defined in echo_sensor.c
}

// ------------------ Wi-Fi Access Point ------------------
#define AP_SSID      "S3_Hub_AP"
#define AP_PASS      "12345678"
#define AP_CHANNEL   6
#define MAX_STA_CONN 4

static const char *TAG = "ESP32-S3_AP";
static const char *TAG_VOICE = "VOICE";


// -------- Voice confidence thresholds --------
#define CONF_THR_MOVE   0.75f   // Forward/Backward/Left/Right must be >= 0.80
#define CONF_THR_STOP   0.50f   // Stop can be more permissive (safer)

// Make these arrays file-scope so both init + presence task can use them
static const gpio_num_t kTrigPins[6] = { TRIGGER1, TRIGGER2, TRIGGER3, TRIGGER4, TRIGGER5, TRIGGER6 };
static const gpio_num_t kEchoPins[6] = { ECHO1,    ECHO2,    ECHO3,    ECHO4,    ECHO5,    ECHO6    };

// ---- Sonar print helper config ----
#define OBSTACLE_NEAR_M   0.60f   // <= this = obstacle
#define OBSTACLE_CLEAR_M  0.80f   // >= this = considered clear (hysteresis)
#define SONAR_REMIND_MS   2000    // reprint status every 2s if unchanged

// Name your sensors (adjust order to match TRIG/ECHO indexing)
static const char* kSonarName[6] = {
    "S1", "S2", "S3", "S4", "S5", "S6"
    // e.g., "Front-Left","Front","Front-Right","Left","Right","Rear"
};

typedef struct {
    volatile bool        obstacle;     // debounced state
    volatile float       last_dist_m;  // last measured distance
    volatile TickType_t  last_print;   // last time we printed
} SonarState;

static SonarState g_sonar[6] = {0};

// ------------------ E-STOP (momentary hold) ------------------
#define ESTOP_PIN           GPIO_NUM_41
#define ESTOP_DEBOUNCE_MS   30
static volatile bool     estop_pressed = false;     // active-low while button held
static volatile uint32_t estop_last_isr_ticks = 0;  // debounce

static void pwm_all_off(void);   // fwd
static void estop_force_stop_now(void);
static inline bool estop_is_pressed(void) { return estop_pressed; }

// ------------------ Motor PWM ------------------
static const gpio_num_t kAllTrans[] = {
  TRANS1, TRANS2, TRANS3, TRANS4, TRANS5, TRANS6, TRANS7, TRANS8
};

#define PWM_FREQ_HZ     16000
#define PWM_RES_BITS    12
#define PWM_RES_ENUM    LEDC_TIMER_12_BIT
#define PWM_MAX_DUTY    ((1U << PWM_RES_BITS) - 1)
#define SPEED_DUTY_PCT  75

static const TickType_t BRAKE_MS = pdMS_TO_TICKS(400);
static ledc_channel_config_t s_ch[8];
typedef enum { CMD_NONE=0, CMD_FORWARD, CMD_BACK, CMD_LEFT, CMD_RIGHT, CMD_STOP } Command;

// Unified state
static Command current_cmd = CMD_NONE;
static Command last_cmd_manual = CMD_NONE;
static Command last_cmd_voice = CMD_NONE;

#define LOG_BUFFER_SIZE 8192
static char log_buffer[LOG_BUFFER_SIZE];
static size_t log_write_pos = 0;


// ---------- Voice latch state ----------
static volatile bool   voice_active   = false;                    // true while voice is driving motion
static TickType_t      last_voice_tick = 0;                       // last time we heard a valid voice cmd
static const TickType_t VOICE_TIMEOUT  = pdMS_TO_TICKS(15000);    // 15 s silence → STOP

// ------------------ PWM Helpers ------------------
static void pwm_init(void) {
  ledc_timer_config_t t{};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = PWM_RES_ENUM;
  t.timer_num       = LEDC_TIMER_0;
  t.freq_hz         = PWM_FREQ_HZ;
#if defined(LEDC_AUTO_CLK)
  t.clk_cfg         = LEDC_AUTO_CLK;
#else
  t.clk_cfg         = LEDC_USE_APB_CLK;
#endif
  ESP_ERROR_CHECK(ledc_timer_config(&t));

  for (int i = 0; i < 8; i++) {
      ledc_channel_config_t ch{};
      ch.gpio_num   = kAllTrans[i];
      ch.speed_mode = LEDC_LOW_SPEED_MODE;
      ch.channel    = static_cast<ledc_channel_t>(LEDC_CHANNEL_0 + i);
      ch.timer_sel  = LEDC_TIMER_0;
      ch.duty       = 0;
      ch.hpoint     = 0;
      ch.intr_type  = LEDC_INTR_DISABLE;
#if ESP_IDF_VERSION_MAJOR >= 5
      ch.flags.output_invert = 0;
#endif
      s_ch[i] = ch;
      ESP_ERROR_CHECK(ledc_channel_config(&s_ch[i]));
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
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint32_t)((PWM_MAX_DUTY * (uint64_t)pct) / 100ULL);
}

// ------------------ Drive Function ------------------
static void drive_pwm(Command cmd, uint32_t duty){
  pwm_all_off();
  esp_rom_delay_us(400);
  switch (cmd) {
      case CMD_FORWARD: pwm_set_duty_idx(0,duty); pwm_set_duty_idx(3,duty); pwm_set_duty_idx(5,duty); pwm_set_duty_idx(6,duty); break;
      case CMD_BACK:    pwm_set_duty_idx(1,duty); pwm_set_duty_idx(2,duty); pwm_set_duty_idx(4,duty); pwm_set_duty_idx(7,duty); break;
      case CMD_LEFT:    pwm_set_duty_idx(1,duty); pwm_set_duty_idx(2,duty); pwm_set_duty_idx(5,duty); pwm_set_duty_idx(6,duty); break;
      case CMD_RIGHT:   pwm_set_duty_idx(0,duty); pwm_set_duty_idx(3,duty); pwm_set_duty_idx(4,duty); pwm_set_duty_idx(7,duty); break;
      default: break;
  }
}

// -------------- E-STOP helpers --------------
static void estop_force_stop_now(void) {
    pwm_all_off();
    current_cmd     = CMD_STOP;
    last_cmd_manual = CMD_NONE;
    voice_active    = false;
    last_cmd_voice  = CMD_NONE;
}

static void IRAM_ATTR estop_isr(void *) {
    const TickType_t now = xTaskGetTickCountFromISR();
    if ((now - estop_last_isr_ticks) > pdMS_TO_TICKS(ESTOP_DEBOUNCE_MS)) {
        const bool level_low = (gpio_get_level(ESTOP_PIN) == 0);
        estop_pressed = level_low;
        if (level_low) {
            estop_force_stop_now();
            ESP_EARLY_LOGW(TAG, "E-STOP pressed -> MOTORS OFF");
        } else {
            ESP_EARLY_LOGI(TAG, "E-STOP released -> control may resume");
        }
        estop_last_isr_ticks = now;
    }
}

static void estop_init(void) {
    gpio_config_t cfg{};
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pin_bit_mask = (1ULL << ESTOP_PIN);
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;      // pull-up so button to GND works
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_ANYEDGE;        // press+release
    ESP_ERROR_CHECK(gpio_config(&cfg));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ESTOP_PIN, estop_isr, nullptr));

    estop_pressed = (gpio_get_level(ESTOP_PIN) == 0);
    if (estop_pressed) {
        estop_force_stop_now();
        ESP_LOGW(TAG, "E-STOP low at boot; holding STOP while pressed");
    } else {
        ESP_LOGI(TAG, "E-STOP ready on GPIO%d (active-low, momentary)", (int)ESTOP_PIN);
    }
}

// ------------------ SENSOR GPIO INIT ------------------
static void sonar_gpio_init(void) {
    gpio_config_t out_cfg{};
    out_cfg.mode = GPIO_MODE_OUTPUT;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;

    gpio_config_t in_cfg{};
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // Triggers
    for (int i = 0; i < 6; i++) {
        out_cfg.pin_bit_mask = (1ULL << kTrigPins[i]);
        ESP_ERROR_CHECK(gpio_config(&out_cfg));
        gpio_set_level(kTrigPins[i], 0);
    }

    // Echoes
    for (int i = 0; i < 6; i++) {
        in_cfg.pin_bit_mask = (1ULL << kEchoPins[i]);
        ESP_ERROR_CHECK(gpio_config(&in_cfg));
    }

    ESP_LOGI(TAG, "Sonar GPIO initialized");
}


// Unified update (manual > voice), with SENSOR SAFETY on voice only
static void update_wheelchair(Command new_cmd, bool from_manual){
    if (estop_is_pressed() && current_cmd != CMD_STOP) {
            drive_pwm(CMD_STOP, 0);
            current_cmd = CMD_STOP;
        return;
    }

    // If this is NOT manual (i.e., voice), block motion on fault only if sensors are connected
    if (!from_manual && sensor_fault && (new_cmd != CMD_STOP && new_cmd != CMD_NONE)) {
        if (current_cmd != CMD_STOP) {
            ESP_LOGW(TAG, "SENSOR FAULT — voice blocked, forcing STOP");
            drive_pwm(CMD_STOP, 0);
            current_cmd = CMD_STOP;
        }
        return;
    }

    if (new_cmd == current_cmd) {
        //ESP_LOGW(TAG, "Before return statement");
        return;
    } 

   // Brake between direction changes
   drive_pwm(CMD_STOP, 0);
   vTaskDelay(BRAKE_MS);

    if (new_cmd != CMD_NONE && new_cmd != CMD_STOP) {
        drive_pwm(new_cmd, duty_from_pct(SPEED_DUTY_PCT));
    }

   current_cmd = new_cmd;
}

// ------------------ Wi-Fi SoftAP ------------------
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
  if (event_id == WIFI_EVENT_AP_STACONNECTED){
      auto *e = (wifi_event_ap_staconnected_t*)event_data;
      ESP_LOGI(TAG,"Device " MACSTR " joined AID=%d", MAC2STR(e->mac), e->aid);
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED){
      auto *e = (wifi_event_ap_stadisconnected_t*)event_data;
      ESP_LOGI(TAG,"Device " MACSTR " left AID=%d", MAC2STR(e->mac), e->aid);
  }
}

static void wifi_init_softap(void){
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  (void)esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));

  wifi_config_t config{};
  strncpy((char*)config.ap.ssid, AP_SSID, sizeof(config.ap.ssid)-1);
  config.ap.ssid_len = strlen(AP_SSID);
  strncpy((char*)config.ap.password, AP_PASS, sizeof(config.ap.password)-1);
  config.ap.channel = AP_CHANNEL;
  config.ap.max_connection = MAX_STA_CONN;
  config.ap.authmode = (strlen(AP_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
  config.ap.ssid_hidden = 0;
  config.ap.beacon_interval = 100;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG,"AP started – SSID:%s PASS:%s", AP_SSID, AP_PASS);
}

// ------------------ HTTP Server ------------------
static esp_err_t status_get_handler(httpd_req_t *req);  // <— add this line
static esp_err_t command_get_handler(httpd_req_t *req)  // keep as-is below
{
    char buf[32];
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len > sizeof(buf)) len = sizeof(buf);

    if (httpd_req_get_url_query_str(req, buf, len) != ESP_OK) {
        httpd_resp_sendstr(req, "Invalid");
        return ESP_FAIL;
    }

    Command cmd = CMD_NONE;
    if      (strstr(buf,"forward"))  cmd = CMD_FORWARD;
    else if (strstr(buf,"backward")) cmd = CMD_BACK;
    else if (strstr(buf,"left"))     cmd = CMD_LEFT;
    else if (strstr(buf,"right"))    cmd = CMD_RIGHT;
    else if (strstr(buf,"stop"))     cmd = CMD_STOP;

    if(cmd != last_cmd_manual){
        last_cmd_manual = cmd;
        ESP_LOGI(TAG,"Manual Command Received: %d", cmd);
        update_wheelchair(cmd, /*from_manual=*/true);

        char response[64];
        snprintf(response,sizeof(response),"OK CMD=%d PWM=%u", cmd, duty_from_pct(SPEED_DUTY_PCT));
        httpd_resp_sendstr(req,response);
    } else {
        httpd_resp_sendstr(req,"No change");
    }

   return ESP_OK;
}


// TO THIS:
static int my_log_handler(const char *fmt, va_list args)
{
    char msg[256];

    // Copy the va_list so we can use it twice
    va_list args_copy;
    va_copy(args_copy, args);

    int len = vsnprintf(msg, sizeof(msg), fmt, args_copy);
    va_end(args_copy);

    if (len > 0) {
        if (len > (int)sizeof(msg)) {
            len = sizeof(msg);
        }
        for (int i = 0; i < len; ++i) {
            log_buffer[log_write_pos] = msg[i];
            log_buffer[log_write_pos] = msg[i];
            log_write_pos = (log_write_pos + 1) % LOG_BUFFER_SIZE;
        }
    }

    // Print to UART so Serial Monitor still works
    int written = vprintf(fmt, args);
    return written;
}


static esp_err_t direct_command_handler(httpd_req_t *req) {
    const char *uri = req->uri;  // e.g., "/forward"
    Command cmd = CMD_NONE;

    if (strcasecmp(uri, "/forward") == 0)  cmd = CMD_FORWARD;
    else if (strcasecmp(uri, "/backward") == 0) cmd = CMD_BACK;
    else if (strcasecmp(uri, "/left") == 0)     cmd = CMD_LEFT;
    else if (strcasecmp(uri, "/right") == 0)    cmd = CMD_RIGHT;
    else if (strcasecmp(uri, "/stop") == 0)     cmd = CMD_STOP;

    if(cmd != CMD_NONE){
        last_cmd_manual = cmd;
        ESP_LOGI(TAG,"Manual Command Received: %d", cmd);
        update_wheelchair(cmd, /*from_manual=*/true);

        char response[64];
        snprintf(response, sizeof(response), "OK CMD=%d PWM=%u", cmd, duty_from_pct(SPEED_DUTY_PCT));
        httpd_resp_sendstr(req, response);
    } else {
        httpd_resp_sendstr(req, "Unknown command");
    }


    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    char json[256];
    snprintf(json, sizeof(json),
            "{\"motorsActive\":%s,\"currentCmd\":%d,\"pwm\":%lu,"
            "\"sensorFault\":%s,\"estopPressed\":%s}",
            (current_cmd != CMD_NONE && current_cmd != CMD_STOP) ? "true" : "false",
            current_cmd,
            (unsigned long)duty_from_pct(SPEED_DUTY_PCT),
            sensor_fault ? "true" : "false",
            estop_is_pressed() ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}



static esp_err_t logs_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");

    size_t pos = log_write_pos;

    // Stream logs in order
    for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
        char c = log_buffer[(pos + i) % LOG_BUFFER_SIZE];
        httpd_resp_send_chunk(req, &c, 1);
    }

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ===========================================================
// Auto-refreshing monitor HTML page
// ===========================================================
static const char *MONITOR_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>ESP32 Monitor</title>
  <style>
    body {
      font-family: monospace;
      background: #111;
      color: #0f0;
      margin: 0;
      padding: 0;
    }
    #log {
      white-space: pre-wrap;
      padding: 8px;
    }
  </style>
</head>
<body>
  <div id="log">Loading...</div>
  <script>
    async function fetchLogs() {
      try {
        const res = await fetch('/monitor', { cache: 'no-store' });
        const text = await res.text();
        const logDiv = document.getElementById('log');
        logDiv.textContent = text;
        window.scrollTo(0, document.body.scrollHeight);
      } catch (e) {
        console.error(e);
      }
    }

    // Poll every 1 second
    fetchLogs();
    setInterval(fetchLogs, 1000);
  </script>
</body>
</html>
)HTML";


static esp_err_t monitor_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, MONITOR_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = nullptr;

    if(httpd_start(&server, &config) == ESP_OK){
        httpd_uri_t uri = { .uri="/command", .method=HTTP_GET, .handler=command_get_handler, .user_ctx=nullptr };
        httpd_register_uri_handler(server, &uri);

        httpd_uri_t status_uri = { .uri="/status", .method=HTTP_GET, .handler=status_get_handler, .user_ctx=nullptr };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t logs_uri = {
            .uri       = "/logs",
            .method    = HTTP_GET,
            .handler   = logs_get_handler,
            .user_ctx  = nullptr
        };
        httpd_register_uri_handler(server, &logs_uri);

        // NEW: alias /monitor → same handler as /logs
        httpd_uri_t monitor_uri = {
            .uri       = "/monitor",
            .method    = HTTP_GET,
            .handler   = logs_get_handler,
            .user_ctx  = nullptr
        };
        httpd_register_uri_handler(server, &monitor_uri);

        httpd_uri_t console_uri = {
            .uri      = "/console",
            .method   = HTTP_GET,
            .handler  = monitor_page_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &console_uri);


        const char* dirs[] = {"/forward", "/backward", "/left", "/right", "/stop"};
        for(int i = 0; i < 5; i++){
            httpd_uri_t cmd_uri = { .uri=dirs[i], .method=HTTP_GET, .handler=direct_command_handler, .user_ctx=nullptr };
            httpd_register_uri_handler(server, &cmd_uri);
        }
    }

    return server;
}


// ------------------ I2S Audio + Edge Impulse ------------------
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define AUDIO_BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];
static float   features_buffer[AUDIO_BUFFER_SIZE];

static esp_err_t i2s_init_audio(void){
   i2s_config_t c{};
   c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
   c.sample_rate = I2S_SAMPLE_RATE;
   c.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
   c.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
   c.communication_format = I2S_COMM_FORMAT_I2S_MSB;
   c.intr_alloc_flags = 0;
   c.dma_buf_count = 4;
   c.dma_buf_len = 256;
   c.use_apll = false;
#if ESP_IDF_VERSION_MAJOR>=5
   c.mclk_multiple=I2S_MCLK_MULTIPLE_256; c.fixed_mclk=0;
#endif
   i2s_pin_config_t p{}; p.bck_io_num=GPIO_NUM_5; p.ws_io_num=GPIO_NUM_4; p.data_out_num=I2S_PIN_NO_CHANGE; p.data_in_num=GPIO_NUM_6;

   ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT,&c,0,nullptr));
   return i2s_set_pin(I2S_PORT,&p);
}

static int raw_feature_get_data(size_t offset,size_t length,float* out_ptr){
   memcpy(out_ptr, features_buffer+offset,length*sizeof(float));
   return 0;
}

static inline float threshold_for(Command c) {
    switch (c) {
        case CMD_STOP:    return CONF_THR_STOP;
        case CMD_FORWARD:
        case CMD_BACK:
        case CMD_LEFT:
        case CMD_RIGHT:   return CONF_THR_MOVE;
        default:          return 1.0f; // CMD_NONE -> never passes
    }
}

extern "C" void sonar_observation(int idx, float dist_m) {
    if (idx < 0 || idx >= 6) return;

    // Normalize impossible readings
    const bool valid = (dist_m > 0.02f && dist_m < 6.0f); // 2 cm .. 6 m (tweak as needed)
    TickType_t now = xTaskGetTickCount();

    // Decide desired state with hysteresis
    bool want_obstacle = false;
    if (valid) {
        if (!g_sonar[idx].obstacle) {
            // currently clear -> only set if "near"
            want_obstacle = (dist_m <= OBSTACLE_NEAR_M);
        } else {
            // currently obstacle -> stay until it is "clear" distance
            want_obstacle = !(dist_m >= OBSTACLE_CLEAR_M);
        }
    } else {
        // Invalid reading: keep current state, but allow reminders
        want_obstacle = g_sonar[idx].obstacle;
    }

    // Edge change?
    if (want_obstacle != g_sonar[idx].obstacle) {
        g_sonar[idx].obstacle = want_obstacle;
        g_sonar[idx].last_print = now;

        if (want_obstacle) {
            ESP_LOGW("SONAR", "[%s] OBSTACLE at %.2f m", kSonarName[idx], dist_m);
        } else {
            ESP_LOGI("SONAR", "[%s] CLEAR (%.2f m)", kSonarName[idx], dist_m);
        }
    } else {
        // Periodic reminder (rate-limited)
        if ((now - g_sonar[idx].last_print) > pdMS_TO_TICKS(SONAR_REMIND_MS)) {
            g_sonar[idx].last_print = now;
            if (g_sonar[idx].obstacle) {
                ESP_LOGW("SONAR", "[%s] still BLOCKED ~%.2f m", kSonarName[idx], dist_m);
            } else {
                if (valid)
                    ESP_LOGI("SONAR", "[%s] still CLEAR ~%.2f m", kSonarName[idx], dist_m);
                else
                    ESP_LOGI("SONAR", "[%s] no valid echo", kSonarName[idx]);
            }
        }
    }

    g_sonar[idx].last_dist_m = valid ? dist_m : -1.0f;
}


static void voice_task(void*) {
    ei_impulse_result_t result;
    const uint32_t duty = duty_from_pct(SPEED_DUTY_PCT);
    ESP_LOGI(TAG_VOICE, "Voice control ready\r\n");

    // initialize as timed-out (no voice command yet)
    last_voice_tick = xTaskGetTickCount();
    voice_active    = false;
    last_cmd_voice  = CMD_NONE;

    while (true) {

        // Immediate safety: while E-STOP is pressed, hold STOP and skip control
        if (estop_is_pressed()) {
            if (current_cmd != CMD_STOP) {
                update_wheelchair(CMD_STOP, /*from_manual=*/false);
            }
            voice_active   = false;
            last_cmd_voice = CMD_NONE;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        // =============== Immediate safety / override checks =================
        // 1) If app/manual control is active, release voice latch (app owns outputs)
        if (last_cmd_manual != CMD_NONE && voice_active) {
            voice_active   = false;     // stop steering outputs via voice
            // Don't force STOP here—manual might be intentionally moving.
            ESP_LOGI(TAG_VOICE, "Voice latch released due to manual override\r\n");
        }

        // 2) If sensors fault while voice is in control, force STOP + clear latch
        if (sensor_fault && voice_active && last_cmd_manual == CMD_NONE) {
            voice_active   = false;
            last_cmd_voice = CMD_NONE;
            update_wheelchair(CMD_STOP, /*from_manual=*/false);   // <-- FIX: 2 args
            ESP_LOGI(TAG_VOICE, "Voice blocked: SENSOR_FAULT → STOP\r\n");
            // keep looping; we will wait for a fresh valid command after fault clears
        }

        // ==================  Audio capture + feature prep  ==================
        size_t bytes_read = 0;
        if (i2s_read(I2S_PORT, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY) != ESP_OK) {
            continue;
        }
        for (size_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
            features_buffer[i] = (float)audio_buffer[i] / 32768.0f;
        }

        signal_t sig{};
        sig.total_length = AUDIO_BUFFER_SIZE;
        sig.get_data     = &raw_feature_get_data;

        if (run_classifier(&sig, &result, false) != EI_IMPULSE_OK) {
            continue;
        }

        ESP_LOGI("PREDICT", "----- Inference Results -----");
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            ESP_LOGI("PREDICT", "  %s : %.4f",
                result.classification[i].label,
                result.classification[i].value
            );
        }

        // =================== Top label + confidence pick ====================
        Command detected = CMD_NONE;
        float   conf     = 0.f;

        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            float val = result.classification[i].value;
            const char* lbl = ei_classifier_inferencing_categories[i];

            if (val > conf) {
                conf = val;
                if      (!strcmp(lbl,"Forward"))  detected = CMD_FORWARD;
                else if (!strcmp(lbl,"Backward")) detected = CMD_BACK;
                else if (!strcmp(lbl,"Left"))     detected = CMD_LEFT;
                else if (!strcmp(lbl,"Right"))    detected = CMD_RIGHT;
                else if (!strcmp(lbl,"Stop"))     detected = CMD_STOP;
                else                              detected = CMD_NONE;
            }
        }

        // ================ Threshold gate (ignore weak hits) =================
        if (detected != CMD_NONE && conf < threshold_for(detected)) {
            detected = CMD_NONE; // treat as no new command this frame
        }

        // =================== Latch behavior / state machine =================
        if (detected == CMD_STOP) {
            // Explicit Stop always wins: clear latch and stop (unless manual is active)
            voice_active   = false;
            last_cmd_voice = CMD_NONE;
            if (last_cmd_manual == CMD_NONE) {
                update_wheelchair(CMD_STOP, /*from_manual=*/false);
            }
            ESP_LOGE(TAG_VOICE, "Voice Command: STOP (%.2f) → STOP\r\n", conf);
        }
        else if (detected != CMD_NONE) {

            // We heard a valid directional command: refresh the timeout clock
            last_voice_tick = xTaskGetTickCount();

            // Only change motion if it’s actually a new command (avoid PWM glitches)
            if (detected != last_cmd_voice) {
                last_cmd_voice = detected;
                voice_active   = true;

                // Voice only drives when there is no manual/app override
                if (!sensor_fault) {
                    update_wheelchair(detected, /*from_manual=*/false);
                    ESP_LOGE("VOICE", "Voice Command: %d (%.2f) → %d", detected, conf, detected);
                } else if (sensor_fault) {
                    voice_active   = false;
                    last_cmd_voice = CMD_NONE;
                    ESP_LOGI(TAG_VOICE, "Voice cmd gated by SENSOR_FAULT\r\n");
                }
            }
            // If detected == last_cmd_voice, we still refreshed last_voice_tick above,
            // so the 15 s timer is extended without toggling outputs.
        }
        else {
            // No valid command this frame — do NOT send CMD_NONE.
            // Latch remains as-is. Timeout below will handle idling out.
        }

        // ================= Timeout: 15 s of silence → STOP ==================
        if (voice_active && last_cmd_manual == CMD_NONE) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_voice_tick) > VOICE_TIMEOUT) {
                voice_active   = false;
                last_cmd_voice = CMD_NONE;
                update_wheelchair(CMD_STOP, /*from_manual=*/false);
                ESP_LOGI(TAG_VOICE, "Voice timeout (15s) → STOP\r\n");
            }
        }
    }
}

// ------------------ Main ------------------
extern "C" void app_main(void) {
   esp_err_t rc = nvs_flash_init();
   if(rc==ESP_ERR_NVS_NO_FREE_PAGES || rc==ESP_ERR_NVS_NEW_VERSION_FOUND) {
       ESP_ERROR_CHECK(nvs_flash_erase());
       ESP_ERROR_CHECK(nvs_flash_init());
   }

   esp_log_set_vprintf(my_log_handler);

   wifi_init_softap();
   pwm_init();
   sonar_gpio_init();                       // <-- NEW: set up sonar pins
   estop_init();
   
   if(i2s_init_audio()!=ESP_OK){
       ESP_LOGE(TAG,"I2S init failed");
       return;
   }

   start_webserver();
   ESP_LOGI(TAG,"Voice + Manual + Wi-Fi + Sensors ready");

   // Start tasks
   xTaskCreate(voice_task, "voice_task", 16000, nullptr, 5, nullptr);
   xTaskCreate(echo_sensor, "echo_sensor", 4096, nullptr, 6, nullptr); // <-- NEW: sensor task

}