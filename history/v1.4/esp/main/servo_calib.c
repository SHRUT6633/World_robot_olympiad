#include "driver/ledc.h"

#define SERVO_PIN GPIO_NUM_7

void servo_sweep(void) {
    ledc_timer_config_t timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_1,
        .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&timer);
    ledc_channel_config_t chan = { .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1, .duty = 0 };
    ledc_channel_config(&chan);

    printf("Servo calibration sweep (-30 to +30 deg)\n");
    for (int deg = -30; deg <= 30; deg += 5) {
        uint32_t pulse = 1638 + (deg + 30) * 27;  // 14-bit: 1638=0°, +27 per deg
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pulse);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        printf("  Angle: %3d°  Pulse: %d\n", deg, pulse);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("Sweep complete\n");
}
