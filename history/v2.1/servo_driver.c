#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_PWM_GPIO   13
#define SERVO_PWM_FREQ   50
#define SERVO_MIN_PULSE  1000
#define SERVO_MAX_PULSE  2000

void servo_init(void) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_0B, SERVO_PWM_GPIO);
    mcpwm_config_t pwm_config = {
        .frequency = SERVO_PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

void servo_set_pulsewidth(int pulse_us) {
    if (pulse_us < SERVO_MIN_PULSE) pulse_us = SERVO_MIN_PULSE;
    if (pulse_us > SERVO_MAX_PULSE) pulse_us = SERVO_MAX_PULSE;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, pulse_us);
}
