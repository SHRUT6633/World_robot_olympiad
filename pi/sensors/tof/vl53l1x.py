import time
from ..base import SensorBase, FilteredSensorMixin
from ...system.logger import log

try:
    from gpiozero import OutputDevice
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class VL53L1X(SensorBase, FilteredSensorMixin):
    def __init__(self, name, bus=1, address=0x32, xshut_pin=None):
        SensorBase.__init__(self, name)
        FilteredSensorMixin.__init__(self, window_size=5)
        self.bus = bus
        self.address = address
        self.xshut_pin = xshut_pin
        self._xshut = None
        self._device = None

    def init(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus)

            if self.xshut_pin is not None and GPIO_AVAILABLE:
                self._xshut = OutputDevice(self.xshut_pin, initial_value=False)
                time.sleep(0.01)
                self._xshut.on()
                time.sleep(0.01)
                self._program_address()

            self._device = True
            log.info(f"{self.name}: VL53L1X @ 0x{self.address:02X} initialized")
        except ImportError:
            log.warn(f"{self.name}: smbus2 not available, using mock")
            self._device = "mock"
        except Exception as e:
            log.warn(f"{self.name}: init error - {e}")
            self._device = "mock"

    def _program_address(self):
        if self.address == 0x29:
            return
        try:
            self._bus.write_byte_data(0x29, 0x8A, self.address << 1)
            time.sleep(0.01)
            log.info(f"{self.name}: address programmed 0x29 -> 0x{self.address:02X}")
        except Exception as e:
            log.warn(f"{self.name}: address programming failed - {e}")

    def read_raw(self):
        if self._device == "mock":
            import random
            return random.uniform(100, 3000)
        try:
            data = self._bus.read_i2c_block_data(self.address, 0x00, 17)
            range_mm = (data[14] << 8) | data[15]
            status = data[16]
            if status == 0:
                return float(range_mm)
            return None
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
        if self._xshut:
            try:
                self._xshut.close()
            except Exception:
                pass
