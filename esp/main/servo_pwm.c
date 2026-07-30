/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/servo_pwm.c
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Servo PWM signal generation
 * ===========================================================================
 */

#include "servo_pwm.h"
#include "driver/ledc.h"
#include "esp_log.h"

/* Tag used for ESP_LOG* messages to identify this module. */
static const char *TAG = "SERVO";

/* Servo PWM configuration constants.
 *
 * SERVO_PIN (GPIO 13)     : output pin for the servo signal line.
 *                           Servo power (VCC, GND) must be wired separately.
 * SERVO_TIMER             : LEDC timer index (use timer 0).
 * SERVO_CHANNEL           : LEDC channel index (use channel 0).
 * SERVO_FREQ (50 Hz)      : standard RC servo frame rate (20 ms period).
 *                           Changing this will desynchronise the servo timing.
 * SERVO_RES (12 bits)     : duty resolution (0..4095).
 *                           12 bits at 50 Hz requires ~5 MHz PWM clock.
 * SERVO_MIN_PULSE_US (500): minimum pulse width in microseconds.
 * SERVO_MAX_PULSE_US (2500): maximum pulse width in microseconds.
 * SERVO_CENTER_NS (1500000): centre pulse as nanoseconds (1500 µs).
 * SERVO_MAX_ANGLE (30.0f) : mechanical limit in degrees (±30°).
 *                           Hardware-dependent; exceeding this may
 *                           damage gears or stall the servo.
 */
#define SERVO_PIN          13
#define SERVO_TIMER        LEDC_TIMER_0
#define SERVO_CHANNEL      LEDC_CHANNEL_0
#define SERVO_FREQ         50
#define SERVO_RES          12
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_CENTER_NS    1500000
#define SERVO_MAX_ANGLE    30.0f

/* servo_pwm_init
 * Initialises the LEDC hardware for servo PWM generation.
 *
 * Steps:
 *   1. Configure LEDC timer 0 at 50 Hz with 12-bit resolution.
 *   2. Attach LEDC channel 0 to SERVO_PIN, initial duty = 0.
 *
 * A 50 Hz signal with 12-bit resolution means the PWM counter
 * counts from 0 to 4095 every 20 ms. A 1500 µs neutral pulse
 * corresponds to duty = 4096 * 1500 / 20000 ≈ 307.
 */
void servo_pwm_init(void) {
    /* LEDC timer config */
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RES,   /* 12 bits */
        .freq_hz = SERVO_FREQ,          /* 50 Hz */
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    /* LEDC channel config */
    ledc_channel_config_t channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = SERVO_TIMER,
        .duty = 0,                       /* start neutral/off */
        .hpoint = 0,
    };
    ledc_channel_config(&channel);

    ESP_LOGI(TAG, "Servo PWM on GPIO%d @ %dHz", SERVO_PIN, SERVO_FREQ);
}

/* servo_set_angle
 * Moves the servo to the specified angle.
 *
 * Algorithm:
 *   1. Clamp angle_deg to ±SERVO_MAX_ANGLE.
 *   2. Map angle to pulse-width offset from centre:
 *        offset = (angle / max_angle) * (max_pulse - centre_pulse)
 *      with centre_pulse = SERVO_CENTER_NS / 1000 = 1500 µs.
 *   3. Clamp result to [SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US].
 *   4. Convert pulse width to LEDC duty:
 *        duty = (2^RES) * pulse_us / period_us
 *      where period_us = 1 000 000 / 50 = 20 000 µs.
 *
 * Example at 0°:
 *   pulse_us = 1500, duty = 4096 * 1500 / 20000 ≈ 307
 *
 * The conversion from angle to pulse width is linear. If the servo
 * response is not linear, a lookup table or polynomial would be
 * needed instead.
 */
void servo_set_angle(float angle_deg) {
    /* Clamp to safe mechanical limits. */
    if (angle_deg > SERVO_MAX_ANGLE) angle_deg = SERVO_MAX_ANGLE;
    if (angle_deg < -SERVO_MAX_ANGLE) angle_deg = -SERVO_MAX_ANGLE;

    /* Centre pulse in microseconds (1500 µs). */
    float pulse_us = SERVO_CENTER_NS / 1000.0f;

    /* Add angular offset to the pulse width.
     * When angle = 0, offset = 0 → pulse = 1500 µs.
     * When angle = +30°, offset ≈ +1000 µs → pulse ≈ 2500 µs.
     * When angle = -30°, offset ≈ -1000 µs → pulse ≈ 500 µs.
     */
    pulse_us += (angle_deg / SERVO_MAX_ANGLE) *
                (SERVO_MAX_PULSE_US - SERVO_CENTER_NS / 1000.0f);

    /* Clamp to valid servo pulse range. */
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    /* Convert pulse width to LEDC duty value.
     * period_us = 1 000 000 / freq = 20 000 µs.
     */
    uint32_t period_us = 1000000 / SERVO_FREQ;
    uint32_t duty = (uint32_t)((float)(1 << SERVO_RES) * pulse_us / period_us);

    /* Apply the new duty cycle. */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);

    ESP_LOGD(TAG, "Servo: %.1fdeg -> pulse=%uus duty=%u",
             angle_deg, (unsigned)pulse_us, duty);
}
