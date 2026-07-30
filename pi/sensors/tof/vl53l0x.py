import time
from ..base import SensorBase, FilteredSensorMixin
from ...system.logger import log

try:
    # gpiozero provides a simple interface to Raspberry Pi GPIO pins.
    # OutputDevice controls digital output pins (e.g. XSHUT).
    from gpiozero import OutputDevice
    GPIO_AVAILABLE = True
except ImportError:
    # On non-RPi platforms (Windows/macOS for dev), GPIO is unavailable.
    GPIO_AVAILABLE = False


class VL53L0X(SensorBase, FilteredSensorMixin):
    """
    Driver for the STMicroelectronics VL53L0X time-of-flight ranging sensor.

    Physical meaning:
      The VL53L0X emits a Class 1 eye-safe 940 nm VCSEL laser pulse and
      measures the time it takes for the reflected light to return. From
      the time-of-flight (ToF), it computes the distance to the target.

      This is fundamentally different from ultrasonic or IR triangulation:
      ToF is immune to target colour, texture, and ambient light (within
      reason). It measures the ABSOLUTE distance, not relative intensity.

    I2C details:
      Default address: 0x29 (7-bit).
      After programming via XSHUT, the address can be changed (see
      _program_address()). This allows up to ~15 VL53L0X sensors on the
      same I2C bus by assigning unique addresses.

    Register map (key registers):
      0x00–0x0B : Result registers (12 bytes).
        byte 10 (MSB) and byte 11 (LSB) = 16-bit range in millimetres.
      0x8A      : I2C slave device address register (used for re-addressing).

    Range and accuracy:
      - Range: 30 mm to ~1200 mm (indoors), ~800 mm (outdoors in sunlight).
      - Accuracy: ±3 % at mid-range, ±10 % near maximum range.
      - Sunlight above ~100 klx (direct sun) can swamp the SPAD detector
        and reduce range to ~500 mm.

    Configuration parameters:
      bus       : I2C bus number (1 = /dev/i2c-1 on RPi 3B+/4/5).
      address   : I2C address (0x30–0x3F after reprogramming; 0x29 default).
      xshut_pin : GPIO pin (BCM) connected to the sensor's XSHUT pin.
                  Used for hardware addressing: pulling XSHUT low puts the
                  sensor into hardware standby. The init sequence pulses
                  this pin to wake the sensor, then programs a new I2C address.

    How the robot uses this:
      The VL53L0X is typically used as a short-range obstacle detector
      mounted on the front (or sides) of the robot. The filtered distance
      reading drives:
        - Emergency stop if an obstacle is < threshold mm ahead.
        - Wall-following behaviour for navigation.
        - Measuring gaps for WRO precision-driving tasks.
    """

    def __init__(self, name, bus=1, address=0x30, xshut_pin=None):
        """
        name      : Unique sensor name (e.g. "ToF_front", "ToF_left").
        bus       : I2C bus number. RPi uses bus 1.
        address   : Desired I2C address AFTER reprogramming.
                    Note: 0x29 is the factory default. We use 0x30+ so
                    multiple sensors can coexist.
        xshut_pin : BCM GPIO pin for the XSHUT line. If None, the sensor
                    is assumed to already have a unique address.
        """
        SensorBase.__init__(self, name)
        FilteredSensorMixin.__init__(self, window_size=5)
        self.bus = bus
        self.address = address
        self.xshut_pin = xshut_pin
        self._xshut = None  # gpiozero OutputDevice handle.
        self._device = None  # "mock" (simulated) or truthy (real).

    def init(self):
        """
        Initialise the VL53L0X sensor.

        Procedure:
          1. Open the I2C bus (smbus2.SMBus).
          2. If xshut_pin is provided, pulse XSHUT low → high to wake the
             sensor from hardware standby, then program a new I2C address.
          3. Set self._device to indicate readiness.

        If smbus2 is not installed (development on Windows), or hardware
        init fails, the driver falls back to "mock" mode and generates
        random distances for testing.
        """
        try:
            import smbus2
            # Open the I2C bus. /dev/i2c-1 on RPi.
            self._bus = smbus2.SMBus(self.bus)

            if self.xshut_pin is not None and GPIO_AVAILABLE:
                # Set XSHUT low (sensor in reset / standby).
                self._xshut = OutputDevice(self.xshut_pin, initial_value=False)
                time.sleep(0.01)
                # Set XSHUT high (sensor powered on at default address 0x29).
                self._xshut.on()
                time.sleep(0.01)
                # Program a new address so multiple sensors can share the bus.
                self._program_address()

            # Mark as initialised (real hardware).
            self._device = True
            log.info(f"{self.name}: VL53L0X @ 0x{self.address:02X} initialized")
        except ImportError:
            log.warn(f"{self.name}: smbus2 not available, using mock")
            self._device = "mock"
        except Exception as e:
            # Catch all: hardware not connected, wrong permissions, etc.
            log.warn(f"{self.name}: init error - {e}")
            self._device = "mock"

    def _program_address(self):
        """
        Change the sensor's I2C address from the default 0x29 to self.address.

        The VL53L0X stores its I2C address in register 0x8A. Writing
        (address << 1) sets the new 7-bit address (shifted left by 1 bit
        because the register expects the 8-bit I2C write address).

        This is essential when using multiple VL53L0X sensors on the same
        I2C bus: each sensor must have a unique address. The procedure is:
          1. Hold all sensors in reset (XSHUT low).
          2. Release one sensor, program its address.
          3. Hold it in reset again.
          4. Repeat for each sensor.

        If the target address is 0x29 (same as default), skip (no change
        needed).
        """
        if self.address == 0x29:
            return
        try:
            # Register 0x8A: I2C slave address. Write address << 1 (8-bit format).
            self._bus.write_byte_data(0x29, 0x8A, self.address << 1)
            time.sleep(0.01)
            log.info(f"{self.name}: address programmed 0x29 -> 0x{self.address:02X}")
        except Exception as e:
            log.warn(f"{self.name}: address programming failed - {e}")

    def read_raw(self):
        """
        Read the raw distance measurement from the sensor.

        Protocol:
          1. Perform an I2C block read from register 0x00, 12 bytes.
          2. Bytes 10–11 contain the 16-bit range result (little-endian-ish:
             byte10 = MSB, byte11 = LSB).
          3. Result is in millimetres.

        Mock mode:
          Returns a random value in [50, 800] mm for testing without hardware.

        Returns:
          float distance in mm, or None on read error.

        Note: In real use, this should be called at 30–50 Hz for responsive
        obstacle avoidance. The sensor's internal measurement time is ~20 ms
        (50 Hz) in standard mode.
        """
        if self._device == "mock":
            import random
            # Random distance between 5 cm and 80 cm (typical robot range).
            return random.uniform(50, 800)
        try:
            # Block read from register 0x00, 12 bytes.
            data = self._bus.read_i2c_block_data(self.address, 0x00, 12)
            # Combine byte 10 (MSB) and byte 11 (LSB) into 16-bit distance.
            range_mm = (data[10] << 8) | data[11]
            return float(range_mm)
        except Exception as e:
            log.warn(f"{self.name}: read error - {e}")
            return None

    def read(self):
        """
        High-level read with outlier rejection + moving average filtering.

        Overrides SensorBase.read() to apply:
          1. self.filter_outliers(raw) — remove spurious readings (>3 sigma).
          2. self.filter_moving_avg(filtered) — smooth the signal.

        This two-stage filter prevents:
          - Single-frame glitches (e.g. from EMI, crosstalk) from
            triggering false emergency stops.
          - High-frequency noise from causing PID oscillation in
            wall-following.

        Lag introduced:
          - window_size=5 gives ~100 ms of delay at 50 Hz sample rate,
            which is acceptable for obstacle avoidance at low speed.

        Returns:
          float filtered distance in mm, or None if the raw read failed.
        """
        raw = super().read()
        if raw is None:
            return None
        filtered = self.filter_outliers(raw)
        return self.filter_moving_avg(filtered)

    def close(self):
        """
        Release I2C bus and GPIO resources.

        Must be called during shutdown to prevent:
          - I2C bus lock (SDA/SCL lines stuck).
          - GPIO pin in undefined state.
        """
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
