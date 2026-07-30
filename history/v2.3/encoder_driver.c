#include "driver/pcnt.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCNT_LEFT_UNIT   PCNT_UNIT_0
#define PCNT_RIGHT_UNIT  PCNT_UNIT_1
#define PCNT_LEFT_GPIO   GPIO_NUM_4
#define PCNT_RIGHT_GPIO  GPIO_NUM_5

#define AS5600_ADDR      0x36
#define AS5600_ANGLE_REG 0x0C

static int16_t prev_left_raw = 0;
static int16_t prev_right_raw = 0;

static void pcnt_init(void) {
    pcnt_config_t left = {
        .pulse_gpio_num = PCNT_LEFT_GPIO,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .unit = PCNT_LEFT_UNIT,
        .channel = PCNT_CHANNEL_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    pcnt_unit_config(&left);

    pcnt_config_t right = {
        .pulse_gpio_num = PCNT_RIGHT_GPIO,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .unit = PCNT_RIGHT_UNIT,
        .channel = PCNT_CHANNEL_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    pcnt_unit_config(&right);

    pcnt_counter_pause(PCNT_LEFT_UNIT);
    pcnt_counter_clear(PCNT_LEFT_UNIT);
    pcnt_counter_resume(PCNT_LEFT_UNIT);

    pcnt_counter_pause(PCNT_RIGHT_UNIT);
    pcnt_counter_clear(PCNT_RIGHT_UNIT);
    pcnt_counter_resume(PCNT_RIGHT_UNIT);
}

static int16_t as5600_read_angle(void) {
    uint8_t buf[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AS5600_ANGLE_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return (buf[0] << 8) | buf[1];
}

void encoder_poll(int16_t *left_delta, int16_t *right_delta) {
    int16_t left_raw = as5600_read_angle();
    *left_delta = left_raw - prev_left_raw;
    prev_left_raw = left_raw;

    int16_t right_raw = as5600_read_angle();
    *right_delta = right_raw - prev_right_raw;
    prev_right_raw = right_raw;
}

void encoder_task(void *pv) {
    pcnt_init();
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
        int16_t ld, rd;
        encoder_poll(&ld, &rd);
    }
}
