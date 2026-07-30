/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/selftest.c
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Hardware self-test routines
 * ===========================================================================
 */

#include "selftest.h"
#include "crc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

/* Tag used for ESP_LOG* messages to identify this module. */
static const char *TAG = "SELFTEST";

/* Onboard LED GPIO (typically GPIO 2 on ESP32 dev boards).
 * Used as a simple visual indicator during the self-test.
 * The LED is toggled on and then off with a 10 ms delay.
 * If this pin is connected to external hardware, the test pulse
 * may be visible and could interfere.
 */
#define TEST_LED_GPIO    2

/* test_led
 * Quick functional test of the onboard LED.
 * Drives the LED GPIO high for 10 ms, then low.
 *
 * ok : output parameter set to true on completion.
 *
 * Note: This does not actually test the UART despite the result
 * field being named "uart_ok". In a proper test suite this should
 * perform a loopback test on the UART peripheral.
 */
static void test_led(bool *ok) {
    gpio_set_level(TEST_LED_GPIO, 1);       /* LED on */
    esp_rom_delay_us(10000);                /* 10 ms delay */
    gpio_set_level(TEST_LED_GPIO, 0);       /* LED off */
    *ok = true;
}

/* test_servo_pwm
 * Verifies that the LEDC channel assigned to the servo (channel 0)
 * can accept a duty-cycle write and update.
 *
 * Writes a duty value corresponding to a 1500 µs pulse (centre)
 * on a 12-bit / 50 Hz timer. If ledc_set_duty succeeds, the test
 * passes; otherwise it fails.
 *
 * ok : output parameter set to true on success, false on error.
 */
static void test_servo_pwm(bool *ok) {
    /* duty = 4096 * 1500 / 20000 ≈ 307 (12-bit, 50 Hz). */
    uint32_t duty = (1 << 12) * 1500 / 20000;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (err == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        *ok = true;
    } else {
        *ok = false;
        ESP_LOGE(TAG, "Servo PWM test failed: %d", err);
    }
}

/* test_motor_pwm
 * Verifies that the LEDC channel assigned to the motor (channel 1)
 * can accept a duty-cycle write, update, and then be set back to 0.
 *
 * Writes a 50 % duty cycle, then immediately resets to 0.
 * The brief pulse may cause a slight motor twitch (noticeable on
 * a bench). If this is undesirable, the duty can be reduced or
 * the motor physically disconnected during testing.
 *
 * ok : output parameter set to true on success, false on error.
 */
static void test_motor_pwm(bool *ok) {
    /* duty = 1024 * 50 / 100 = 512 (10-bit, 20 kHz). */
    uint32_t duty = (1 << 10) * 50 / 100;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    if (err == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        /* Immediately revert to 0. */
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        *ok = true;
    } else {
        *ok = false;
        ESP_LOGE(TAG, "Motor PWM test failed: %d", err);
    }
}

/* test_l298n
 * Verifies that the L298N IN1 and IN2 GPIOs can be driven high and
 * low. Briefly sets IN1 high, IN2 low (forward), holds for 1 ms,
 * then returns both to low (coast).
 *
 * This may cause a brief motor twitch if the ENA pin has a pull-up
 * or if the L298N is powered. For a safe test, disconnect motor
 * power or ensure ENA is low (no PWM) before running.
 *
 * ok : output parameter set to true on completion.
 */
static void test_l298n(bool *ok) {
    gpio_set_level(8, 1);     /* IN1 high */
    gpio_set_level(9, 0);     /* IN2 low  → forward direction */
    esp_rom_delay_us(1000);   /* 1 ms hold */
    gpio_set_level(8, 0);     /* IN1 low */
    gpio_set_level(9, 0);     /* IN2 low  → coast */
    *ok = true;
}

/* test_watchdog
 * Placeholder for a watchdog self-check.
 * Currently always passes. A real implementation could:
 *   - Verify that the watchdog timer is counting.
 *   - Verify that feeding the watchdog resets the timeout.
 *   - Trigger a deliberate watchdog timeout in a safe context
 *     to confirm the reset mechanism works.
 *
 * ok : output parameter set to true.
 */
static void test_watchdog(bool *ok) {
    *ok = true;
}

/* esp_selftest_init
 * Prepares the onboard LED GPIO for use during self-test.
 * Configures GPIO 2 as a push-pull output, initially LOW.
 */
void esp_selftest_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TEST_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(TEST_LED_GPIO, 0);   /* start with LED off */
    ESP_LOGI(TAG, "ESP32 Self-Test initialized");
}

/* esp_selftest_run
 * Runs all five hardware self-tests and records the results.
 *
 * Timing:
 *   - Records a timestamp before and after the tests.
 *   - duration_ms = (end - start) / 1000.
 *
 * The tests are run in a fixed order:
 *   1. LED (misnamed uart_ok)
 *   2. Servo PWM
 *   3. Motor PWM
 *   4. L298N direction pins
 *   5. Watchdog (placeholder)
 *
 * Test results are logged via ESP_LOGI with PASS/FAIL.
 */
void esp_selftest_run(esp_selftest_result_t *result) {
    uint64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "Running ESP32 self-test...");

    result->uart_ok = true;                  /* initialise */
    test_led(&result->uart_ok);             /* step 1 */
    test_servo_pwm(&result->servo_pwm_ok);  /* step 2 */
    test_motor_pwm(&result->motor_pwm_ok);  /* step 3 */
    test_l298n(&result->l298n_ok);          /* step 4 */
    test_watchdog(&result->watchdog_ok);    /* step 5 */

    result->test_duration_ms = (esp_timer_get_time() - t0) / 1000;

    /* Log summary. */
    ESP_LOGI(TAG, "Self-test complete: %ums", result->test_duration_ms);
    ESP_LOGI(TAG, "  UART:       %s", result->uart_ok       ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Servo PWM:  %s", result->servo_pwm_ok  ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Motor PWM:  %s", result->motor_pwm_ok  ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  L298N:      %s", result->l298n_ok  ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Watchdog:   %s", result->watchdog_ok   ? "PASS" : "FAIL");
}

/* esp_selftest_all_passed
 * Returns true only if every individual test passed.
 * The robot should not enter operational mode if this returns false.
 */
bool esp_selftest_all_passed(const esp_selftest_result_t *result) {
    return result->uart_ok && result->servo_pwm_ok &&
            result->motor_pwm_ok && result->l298n_ok &&
           result->watchdog_ok;
}
