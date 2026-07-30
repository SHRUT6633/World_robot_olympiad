import time
from ..base import SensorBase, FilteredSensorMixin
from ...system.logger import log


class VL53L0X(SensorBase, FilteredSensorMixin):
    def __init__(self, name, bus=1, address=0x30):
        SensorBase.__init__(self, name)
        FilteredSensorMixin.__init__(self, window_size=5)
        self.bus = bus
        self.address = address
        self._device = None

    def init(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus)
            self._device = True
            log.info(f"{self.name}: VL53L0X @ 0x{self.address:02X} initialized")
        except ImportError:
            log.warn(f"{self.name}: smbus2 not available, using mock")
            self._device = "mock"

    def read_raw(self):
        if self._device == "mock":
            import random
            return random.uniform(50, 800)
        try:
            data = self._bus.read_i2c_block_data(self.address, 0x00, 12)
            range_mm = (data[10] << 8) | data[11]
            return float(range_mm)
        except Exception as e:
            log.warn(f"{self.name}: read error - {e}")
            return None

    def read(self):
        raw = super().read()
        if raw is None:
            return None
        filtered = self.filter_outliers(raw)
        return self.filter_moving_avg(filtered)

    def close(self):
        if hasattr(self, "_bus") and self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
