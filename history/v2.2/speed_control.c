#include "speed_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "nvs_flash.h"
#include "nvs.h"

#define MOTOR_PWM_GPIO    25
#define MOTOR_DIR1_GPIO   26
#define MOTOR_DIR2_GPIO   27
#define PWM_FREQ          50
#define PWM_RESOLUTION    255
#define DEAD_ZONE          10
#define SOFT_MAX           90
#define RAMP_DELAY_MS      10

static int current_speed = 0;
static int target_speed = 0;

void speed_init(void) {
    mcpwm_config_t cfg = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_0A, MOTOR_PWM_GPIO);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);

    gpio_set_direction(MOTOR_DIR1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DIR2_GPIO, GPIO_MODE_OUTPUT);
}

static int percent_to_duty(int percent) {
    if (percent < DEAD_ZONE) return 0;
    int capped = (percent * SOFT_MAX) / 100;
    int duty = (capped * PWM_RESOLUTION) / 100;
    if (duty > PWM_RESOLUTION) duty = PWM_RESOLUTION;
    return duty;
}

void speed_set_target(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    target_speed = percent;

    if (percent == 0) {
        gpio_set_level(MOTOR_DIR1_GPIO, 0);
        gpio_set_level(MOTOR_DIR2_GPIO, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        current_speed = 0;
        return;
    }

    gpio_set_level(MOTOR_DIR1_GPIO, 1);
    gpio_set_level(MOTOR_DIR2_GPIO, 0);

    int target_duty = percent_to_duty(percent);
    int current_duty = percent_to_duty(current_speed);

    if (target_duty > current_duty) {
        int steps = (target_duty - current_duty) / 5;
        if (steps < 1) steps = 1;
        for (int i = 1; i <= steps; i++) {
            int duty = current_duty + (target_duty - current_duty) * i / steps;
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            vTaskDelay(pdMS_TO_TICKS(RAMP_DELAY_MS));
        }
    }

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, target_duty);
    current_speed = percent;
}

int speed_get_current(void) {
    return current_speed;
}
