#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"

#define MOTOR_PWM_GPIO   25
#define MOTOR_DIR1_GPIO  26
#define MOTOR_DIR2_GPIO  27
#define PWM_FREQ         50
#define RAMP_STEPS       50
#define RAMP_INTERVAL_MS 10

static int current_duty = 0;

static void motor_init(void) {
    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_0A, MOTOR_PWM_GPIO);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    gpio_set_direction(MOTOR_DIR1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DIR2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_DIR1_GPIO, 1);
    gpio_set_level(MOTOR_DIR2_GPIO, 0);
}

static void set_motor_speed(int target_duty) {
    if (target_duty == current_duty) return;

    if (target_duty > current_duty) {
        int steps = (target_duty - current_duty);
        if (steps > RAMP_STEPS) steps = RAMP_STEPS;
        for (int i = 1; i <= steps; i++) {
            int duty = current_duty + (target_duty - current_duty) * i / steps;
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            vTaskDelay(pdMS_TO_TICKS(RAMP_INTERVAL_MS));
        }
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, target_duty);
    current_duty = target_duty;
}

void app_main(void) {
    motor_init();
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

    uint8_t buf[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buf[len] = 0;
            if (strstr((char*)buf, "\"speed\":")) {
                char* p = strstr((char*)buf, "\"speed\":");
                p += 8;
                int speed = atoi(p);
                int duty = (speed * 255) / 100;
                set_motor_speed(duty);
            }
        }
    }
}
