#include "tb6612fng.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "TB6612FNG";

#define PIN_STBY   10
#define PIN_AIN1   8
#define PIN_AIN2   9
#define PIN_BIN1   6
#define PIN_BIN2   7
#define PIN_PWMB   12
#define PIN_PWMA   11

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<PIN_STBY)|(1ULL<<PIN_AIN1)|(1ULL<<PIN_AIN2)|\
                             (1ULL<<PIN_BIN1)|(1ULL<<PIN_BIN2))

void tb6612fng_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_STBY, 1);
    gpio_set_level(PIN_AIN1, 0);
    gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0);
    gpio_set_level(PIN_BIN2, 0);
    ESP_LOGI(TAG, "TB6612FNG initialized");
}

void tb6612fng_set_motor(int speed_pct, bool forward) {
    if (speed_pct < 0) {
        speed_pct = -speed_pct;
        forward = false;
    }
    if (speed_pct > 100) speed_pct = 100;
    gpio_set_level(PIN_AIN1, forward ? 1 : 0);
    gpio_set_level(PIN_AIN2, forward ? 0 : 1);
    uint32_t duty = (uint32_t)speed_pct * 1023 / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
