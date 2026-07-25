import time
import numpy as np
from ..base import SensorBase
from ...system.logger import log


class MPU6050(SensorBase):
    def __init__(self, bus=1, address=0x68, accel_range=4, gyro_range=500):
        super().__init__("MPU6050")
        self.bus = bus
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self._device = None
        self._accel_scale = accel_range / 32768.0
        self._gyro_scale = gyro_range / 32768.0
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

    def init(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus)
            self._bus.write_byte_data(self.address, 0x6B, 0x00)
            self._bus.write_byte_data(self.address, 0x1C, 0x00)
            self._bus.write_byte_data(self.address, 0x1B, 0x00)
            self._bus.write_byte_data(self.address, 0x1A, 0x03)
            self._device = True
            log.info("MPU6050 initialized")
        except ImportError:
            log.warn("MPU6050: smbus2 not available, using mock")
            self._device = "mock"

    def _read_word(self, reg):
        if self._device == "mock":
            import random
            return random.randint(-100, 100)
        try:
            high = self._bus.read_byte_data(self.address, reg)
            low = self._bus.read_byte_data(self.address, reg + 1)
            val = (high << 8) | low
            return val - 65536 if val > 32767 else val
        except Exception as e:
            log.warn(f"MPU6050 read error: {e}")
            return 0

    def read_raw(self):
        ax = self._read_word(0x3B) * self._accel_scale
        ay = self._read_word(0x3D) * self._accel_scale
        az = self._read_word(0x3F) * self._accel_scale
        gx = self._read_word(0x43) * self._gyro_scale
        gy = self._read_word(0x45) * self._gyro_scale
        gz = self._read_word(0x47) * self._gyro_scale
        return {
            "accel": np.array([ax, ay, az]) - self.accel_bias,
            "gyro": np.array([gx, gy, gz]) - self.gyro_bias,
            "accel_raw": np.array([ax, ay, az]),
            "gyro_raw": np.array([gx, gy, gz]),
        }

    def calibrate_gyro_bias(self, samples=200):
        biases = []
        for _ in range(samples):
            data = self.read_raw()
            biases.append(data["gyro_raw"])
            time.sleep(0.003)
        self.gyro_bias = np.mean(biases, axis=0)
        log.info(f"Gyro bias calibrated: {self.gyro_bias}")

    def calibrate_accel_bias(self, samples=100):
        biases = []
        for _ in range(samples):
            data = self.read_raw()
            biases.append(data["accel_raw"])
            time.sleep(0.003)
        self.accel_bias = np.mean(biases, axis=0)
        self.accel_bias[2] -= 9.81
        log.info(f"Accel bias calibrated: {self.accel_bias}")

    def close(self):
        if hasattr(self, "_bus") and self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
