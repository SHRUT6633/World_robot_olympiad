#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMI270_ADDR      0x68
#define BMI270_CHIP_ID   0x00
#define BMI270_GYRO_X_L  0x12
#define BMI270_CMD_REG   0x7E

static float heading = 0.0f;
static float gyro_z = 0.0f;
static TickType_t last_imu_tick = 0;

static void imu_write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}

static void imu_read_gyro(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMI270_GYRO_X_L, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    *x = (buf[1] << 8) | buf[0];
    *y = (buf[3] << 8) | buf[2];
    *z = (buf[5] << 8) | buf[4];
}

void imu_init(void) {
    imu_write_reg(BMI270_CMD_REG, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(50));
    imu_write_reg(0x7C, 0x00);
    imu_write_reg(0x7D, 0x03);
    imu_write_reg(0x02, 0x03);
    last_imu_tick = xTaskGetTickCount();
}

void imu_update(void) {
    TickType_t now = xTaskGetTickCount();
    float dt = (now - last_imu_tick) * portTICK_PERIOD_MS / 1000.0f;
    last_imu_tick = now;

    int16_t gx, gy, gz;
    imu_read_gyro(&gx, &gy, &gz);
    gyro_z = gz * 0.003814f;
    heading += gyro_z * dt;
}

float imu_get_heading(void) {
    return heading;
}

float imu_get_gyro_z(void) {
    return gyro_z;
}
