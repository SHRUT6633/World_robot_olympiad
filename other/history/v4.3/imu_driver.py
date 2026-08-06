import smbus2
import time


MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_Z_OUT = 0x47


class IMUDriver:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

    def read_gyro_z(self):
        high = self.bus.read_byte_data(MPU6050_ADDR, GYRO_Z_OUT)
        low = self.bus.read_byte_data(MPU6050_ADDR, GYRO_Z_OUT + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value / 131.0
