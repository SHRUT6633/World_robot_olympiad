import smbus2
import time


VL53L1X_ADDRESS = 0x29
SOFT_RESET = 0x00
RANGE_START = 0x00
RESULT_RANGE_STATUS = 0x0089


class ToFDriver:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        self.address = VL53L1X_ADDRESS

    def get_distance(self):
        data = self.bus.read_i2c_block_data(
            self.address, RESULT_RANGE_STATUS, 12
        )
        dist_mm = (data[11] << 8) | data[10]
        return dist_mm

    def get_range_status(self):
        data = self.bus.read_i2c_block_data(
            self.address, RESULT_RANGE_STATUS, 12
        )
        status = (data[1] & 0x78) >> 3
        return status

    def start_ranging(self):
        self.bus.write_byte_data(self.address, 0x00, 0x02)
        time.sleep(0.001)

    def stop_ranging(self):
        self.bus.write_byte_data(self.address, 0x00, 0x01)
