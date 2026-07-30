#include "motor_pwm.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

#define MOTOR_PWM_PIN     11
#define MOTOR_TIMER       LEDC_TIMER_1
#define MOTOR_CHANNEL     LEDC_CHANNEL_1
#define MOTOR_FREQ        20000
#define MOTOR_RES         10
#define MOTOR_MAX_DUTY    1023

void motor_pwm_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = MOTOR_TIMER,
        .duty_resolution = MOTOR_RES,
        .freq_hz = MOTOR_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = MOTOR_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = MOTOR_CHANNEL,
        .timer_sel = MOTOR_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&channel);
    ESP_LOGI(TAG, "Motor PWM on GPIO%d @ %dHz", MOTOR_PWM_PIN, MOTOR_FREQ);
}

void motor_set_speed(uint8_t speed_pct) {
    if (speed_pct > 100) speed_pct = 100;
    uint32_t duty = (uint32_t)speed_pct * MOTOR_MAX_DUTY / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_CHANNEL);
    ESP_LOGD(TAG, "Motor: %u%% -> duty=%u", speed_pct, duty);
}
