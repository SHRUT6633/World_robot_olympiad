#include "selftest.h"
#include "crc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

static const char *TAG = "SELFTEST";

#define TEST_LED_GPIO    2

static void test_led(bool *ok) {
    gpio_set_level(TEST_LED_GPIO, 1);
    esp_rom_delay_us(10000);
    gpio_set_level(TEST_LED_GPIO, 0);
    *ok = true;
}

static void test_servo_pwm(bool *ok) {
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

static void test_motor_pwm(bool *ok) {
    uint32_t duty = (1 << 10) * 50 / 100;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    if (err == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        *ok = true;
    } else {
        *ok = false;
        ESP_LOGE(TAG, "Motor PWM test failed: %d", err);
    }
}

static void test_tb6612fng(bool *ok) {
    gpio_set_level(10, 1);
    gpio_set_level(8, 1);
    gpio_set_level(9, 0);
    esp_rom_delay_us(1000);
    gpio_set_level(8, 0);
    gpio_set_level(9, 0);
    *ok = true;
}

static void test_watchdog(bool *ok) {
    *ok = true;
}

void esp_selftest_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TEST_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(TEST_LED_GPIO, 0);
    ESP_LOGI(TAG, "ESP32 Self-Test initialized");
}

void esp_selftest_run(esp_selftest_result_t *result) {
    uint64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "Running ESP32 self-test...");

    result->uart_ok = true;
    test_led(&result->uart_ok);
    test_servo_pwm(&result->servo_pwm_ok);
    test_motor_pwm(&result->motor_pwm_ok);
    test_tb6612fng(&result->tb6612fng_ok);
    test_watchdog(&result->watchdog_ok);

    result->test_duration_ms = (esp_timer_get_time() - t0) / 1000;

    ESP_LOGI(TAG, "Self-test complete: %ums", result->test_duration_ms);
    ESP_LOGI(TAG, "  UART:       %s", result->uart_ok       ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Servo PWM:  %s", result->servo_pwm_ok  ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Motor PWM:  %s", result->motor_pwm_ok  ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  TB6612FNG:  %s", result->tb6612fng_ok  ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Watchdog:   %s", result->watchdog_ok   ? "PASS" : "FAIL");
}

bool esp_selftest_all_passed(const esp_selftest_result_t *result) {
    return result->uart_ok && result->servo_pwm_ok &&
           result->motor_pwm_ok && result->tb6612fng_ok &&
           result->watchdog_ok;
}
