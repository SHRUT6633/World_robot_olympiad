#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_PWM_GPIO    25
#define MOTOR_DIR1_GPIO   26
#define MOTOR_DIR2_GPIO   27

static int braking = 0;

void brake_apply(int duration_ms) {
    braking = 1;
    gpio_set_level(MOTOR_DIR1_GPIO, 0);
    gpio_set_level(MOTOR_DIR2_GPIO, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 255);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    gpio_set_level(MOTOR_DIR1_GPIO, 0);
    gpio_set_level(MOTOR_DIR2_GPIO, 0);
    braking = 0;
}

int brake_is_active(void) {
    return braking;
}

void handle_brake_command(int brake_ms) {
    if (brake_ms > 0) {
        brake_apply(brake_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
