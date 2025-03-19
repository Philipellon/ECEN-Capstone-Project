#include "prototype3.h"  // Ensure your pin definitions are here


void pwm_init(void) {
    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .duty_resolution = PWM_DUTY_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    // Configure PWM for TRANS4 (Forward)
    ledc_channel_config_t pwm_forward = {
        .channel    = PWM_CHANNEL_FORWARD,
        .duty       = 0,  // Start with 0 duty cycle
        .gpio_num   = TRANS4,  // PWM applied here
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_forward));

    // Configure PWM for TRANS3 (Reverse)
    ledc_channel_config_t pwm_reverse = {
        .channel    = PWM_CHANNEL_REVERSE,
        .duty       = 0,  // Start with 0 duty cycle
        .gpio_num   = TRANS3,  // PWM applied here
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_reverse));
}