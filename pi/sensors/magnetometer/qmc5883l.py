import time
import numpy as np
from ..base import SensorBase
from ...system.logger import log


class QMC5883L(SensorBase):
    def __init__(self, bus=1, address=0x0D):
        super().__init__("QMC5883L")
        self.bus = bus
        self.address = address
        self._device = None
        self.hard_iron = np.zeros(3)
        self.soft_iron = np.eye(3)

    def init(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus)
            self._bus.write_byte_data(self.address, 0x09, 0x1D)
            self._bus.write_byte_data(self.address, 0x0B, 0x01)
            self._bus.write_byte_data(self.address, 0x09, 0x1D)
            self._device = True
            log.info("QMC5883L initialized")
        except ImportError:
            log.warn("QMC5883L: smbus2 not available, using mock")
            self._device = "mock"

    def read_raw(self):
        if self._device == "mock":
            import random
            return np.array([random.uniform(-300, 300) for _ in range(3)])
        try:
            data = self._bus.read_i2c_block_data(self.address, 0x00, 6)
            x = (data[1] << 8) | data[0]
            y = (data[3] << 8) | data[2]
            z = (data[5] << 8) | data[4]
            x = x - 65536 if x > 32767 else x
            y = y - 65536 if y > 32767 else y
            z = z - 65536 if z > 32767 else z
            return np.array([x, y, z], dtype=float)
        except Exception as e:
            log.warn(f"QMC5883L read error: {e}")
            return np.zeros(3)

    def read(self):
        raw = super().read()
        if raw is None:
            return None
        compensated = self.soft_iron @ (raw - self.hard_iron)
        return compensated

    def calibrate_hard_iron(self, samples=500):
        mins = np.full(3, np.inf)
        maxs = np.full(3, -np.inf)
        for _ in range(samples):
            val = self.read_raw()
            mins = np.minimum(mins, val)
            maxs = np.maximum(maxs, val)
            time.sleep(0.01)
        self.hard_iron = (maxs + mins) / 2
        log.info(f"Hard iron: {self.hard_iron}")

    def calibrate_soft_iron(self):
        self.soft_iron = np.eye(3)

    def heading(self, mag, accel=None):
        if accel is not None:
            pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
            roll = np.arctan2(accel[1], accel[2])
            x = mag[0] * np.cos(pitch) + mag[2] * np.sin(pitch)
            y = mag[0] * np.sin(roll) * np.sin(pitch) + mag[1] * np.cos(roll) - mag[2] * np.sin(roll) * np.cos(pitch)
        else:
            x, y = mag[0], mag[1]
        return np.degrees(np.arctan2(y, x))

    def close(self):
        if hasattr(self, "_bus") and self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
