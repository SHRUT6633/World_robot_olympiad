#include "servo_pwm.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "SERVO";

#define SERVO_PIN          13
#define SERVO_TIMER        LEDC_TIMER_0
#define SERVO_CHANNEL      LEDC_CHANNEL_0
#define SERVO_FREQ         50
#define SERVO_RES          12
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_CENTER_NS    1500000
#define SERVO_MAX_ANGLE    30.0f

void servo_pwm_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&channel);
    ESP_LOGI(TAG, "Servo PWM on GPIO%d @ %dHz", SERVO_PIN, SERVO_FREQ);
}

void servo_set_angle(float angle_deg) {
    if (angle_deg > SERVO_MAX_ANGLE) angle_deg = SERVO_MAX_ANGLE;
    if (angle_deg < -SERVO_MAX_ANGLE) angle_deg = -SERVO_MAX_ANGLE;

    float pulse_us = SERVO_CENTER_NS / 1000.0f;
    pulse_us += (angle_deg / SERVO_MAX_ANGLE) * (SERVO_MAX_PULSE_US - SERVO_CENTER_NS / 1000.0f);
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    uint32_t period_us = 1000000 / SERVO_FREQ;
    uint32_t duty = (uint32_t)((float)(1 << SERVO_RES) * pulse_us / period_us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);

    ESP_LOGD(TAG, "Servo: %.1fdeg -> pulse=%uus duty=%u", angle_deg, (unsigned)pulse_us, duty);
}
