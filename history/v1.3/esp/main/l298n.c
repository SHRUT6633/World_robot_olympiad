#include "l298n.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "L298N";

#define PIN_ENA   11
#define PIN_IN1   8
#define PIN_IN2   9

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<PIN_IN1)|(1ULL<<PIN_IN2))

void l298n_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = 10,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t chan = {
        .gpio_num = PIN_ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&chan);

    ESP_LOGI(TAG, "L298N initialized (ENA=GPIO11, IN1=GPIO8, IN2=GPIO9)");
}

void l298n_set_motor(int speed_pct, bool forward) {
    if (speed_pct < 0) {
        speed_pct = -speed_pct;
        forward = false;
    }
    if (speed_pct > 100) speed_pct = 100;

    gpio_set_level(PIN_IN1, forward ? 1 : 0);
    gpio_set_level(PIN_IN2, forward ? 0 : 1);

    uint32_t duty = (uint32_t)speed_pct * 1023 / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    ESP_LOGD(TAG, "L298N: speed=%d%% forward=%s", speed_pct, forward ? "yes" : "no");
}
