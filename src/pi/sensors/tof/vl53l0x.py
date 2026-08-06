# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/tof/vl53l0x.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# VL53L0X Time-of-Flight ranging sensor driver
# =============================================================================

import time
from ..base import SensorBase, FilteredSensorMixin
from ...system.logger import log
from . import xshut_manager
from . import family


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
        self._family = "unknown"  # "l0x" / "l1x" / "unknown", set at init.
        # Register this XSHUT pin NOW (at construction). All sensors exist
        # before init_all() runs, so when THIS sensor programs its address,
        # every other sensor's pin is already known and can be held in
        # hardware reset — otherwise the un-initialised sensors would still
        # be powered at 0x29 and receive the address write too.
        xshut_manager.register(xshut_pin)

    def init(self):
        """
        Initialise the VL53L0X sensor.

        Procedure:
          1. Open the I2C bus (smbus2.SMBus).
          2. If xshut_pin is provided, use the shared XSHUT sequencer:
             every other registered ToF sensor is held in hardware reset
             while this one is powered up, so the address write at the
             factory-default 0x29 is only seen by THIS sensor. Without
             this, ALL sensors still at 0x29 would be programmed to the
             same address and corrupt the bus.
          3. Verify the chip actually responds at its programmed address
             (register 0x8A write-back check). If it does not, raise a
             RuntimeError so the SystemManager reports FAILED — never
             silently claim OK.

        If smbus2 is not installed (development on Windows), or the I2C
        bus cannot be opened at all, the driver falls back to "mock" mode
        and generates random distances for testing.
        """
        try:
            import smbus2
        except ImportError:
            log.warn(f"{self.name}: smbus2 not available, using mock")
            self._device = "mock"
            return
        try:
            # Open the I2C bus. /dev/i2c-1 on RPi.
            self._bus = smbus2.SMBus(self.bus)
        except Exception as e:
            log.warn(f"{self.name}: I2C bus unavailable ({e}), using mock")
            self._device = "mock"
            return

        try:
            if self.xshut_pin is not None and xshut_manager.GPIO_AVAILABLE:
                # Hold every other ToF in hardware reset (they are all at
                # 0x29 right now and must not see the address write below).
                held = xshut_manager.hold_all_other_low(self.xshut_pin)
                try:
                    # Release ONLY this sensor.
                    self._xshut = xshut_manager.OutputDevice(
                        self.xshut_pin, initial_value=True)
                    time.sleep(0.05)
                    self._program_address()
                finally:
                    # Power the other sensors back on (they keep their own
                    # already-programmed unique addresses).
                    xshut_manager.release(held)
            elif self.xshut_pin is not None:
                # xshut_pin IS configured but GPIO control is unavailable.
                # This is a misconfiguration, not a silent fallback: without
                # XSHUT the address can never be programmed.
                raise RuntimeError(
                    f"{self.name}: xshut_pin={self.xshut_pin} configured but "
                    f"gpiozero is not installed (run: pip install gpiozero "
                    f"rpi-lgpio) — address programming impossible")
            else:
                # No XSHUT pin: only works if the sensor already has its
                # unique address programmed (e.g. pre-programmed module).
                log.warn(f"{self.name}: no xshut_pin configured — assuming "
                         f"sensor is already at 0x{self.address:02X}")

            # Real hardware present — verify the chip answers at its
            # programmed address before claiming OK.
            if not self.verify():
                raise RuntimeError(
                    f"{self.name}: sensor not responding at "
                    f"0x{self.address:02X} (check wiring, XSHUT pin and "
                    f"I2C address)")
            self._device = True
            self._family = family.detect_family(self._bus, self.address)
            self._start_measurement()
            log.info(f"{self.name}: VL53L0X @ 0x{self.address:02X} initialized")
        except Exception as e:
            log.error(f"{self.name}: init failed - {e}")
            raise

    def verify(self):
        """
        Confirm the chip is present and correctly addressed.

        Reads back the I2C slave address register (0x8A). On a healthy
        chip programmed to self.address it must read back either the
        shifted form (address << 1, genuine ST firmware) or the raw
        form (address, some clone modules). A missing chip, a bus fault,
        or a sensor that is still at the default 0x29 (because XSHUT
        programming failed) will fail this check.
        """
        try:
            got = self._bus.read_byte_data(self.address, 0x8A)
        except Exception as e:
            log.warn(f"{self.name}: verify error - {e}")
            return False
        # Genuine ST chips store address << 1; some clones store the raw
        # address; 0x00 = unconfigured (the chip still answers at its boot
        # address — accepted, the data check still gates real readings).
        if got in (self.address << 1, self.address, 0x00):
            return True
        log.warn(f"{self.name}: address reg 0x{got:02X}, "
                 f"expected 0x{self.address << 1:02X}, "
                 f"0x{self.address:02X} or 0x00")
        return False

    def _find_boot_address(self):
        """
        Locate the sensor's current I2C address after power-up.

        Most VL53L0X modules boot at the factory default 0x29, but some
        clone modules boot at 0x2C instead. This probes both candidates.
        It must be called while every other ToF is held in XSHUT reset,
        so only this sensor can respond and the probe is unambiguous.

        Returns the address that ACKs, or None if no chip is present.
        """
        for candidate in (0x29, 0x2C):
            try:
                self._bus.read_byte_data(candidate, 0x8A)
                return candidate
            except Exception:
                continue
        return None

    def _answers_at(self, addr, expected_reg_8a):
        """
        True if a chip responds at `addr` and its address register (0x8A)
        holds `expected_reg_8a`. During XSHUT programming every other ToF
        is held in reset, so a match at `addr` can only be this sensor.
        """
        try:
            return self._bus.read_byte_data(addr, 0x8A) == expected_reg_8a
        except Exception:
            return False

    def _program_address(self):
        """
        Change the sensor's I2C address from its boot address to self.address.

        The VL53L0X stores its I2C address in register 0x8A. Genuine ST
        chips encode it shifted (value = address << 1); some clone modules
        apply the written value raw as the 7-bit address. Both variants
        are handled here:

          1. Write (address << 1) and check the chip answers at `address`.
          2. If not, the clone variant has moved to `address << 1` — write
             the raw address there, then check `address` again.

        This is essential when using multiple VL53L0X sensors on the same
        I2C bus: each sensor must have a unique address. The procedure is:
          1. Hold all sensors in reset (XSHUT low).
          2. Release one sensor, program its address.
          3. Hold it in reset again.
          4. Repeat for each sensor.

        The chip's current address is located first (0x29, or 0x2C on
        some clone modules) so sensors that boot at a non-standard
        address are still programmed correctly.

        If the target address is 0x29 (same as default), skip (no change
        needed).
        """
        if self.address == 0x29:
            return
        current = self._find_boot_address()
        if current is None:
            log.warn(f"{self.name}: no chip found at boot addresses "
                     f"0x29/0x2C — address programming impossible")
            return
        # Attempt 1: standard ST encoding (address << 1).
        try:
            # Register 0x8A: I2C slave address. Write address << 1 (8-bit format).
            self._bus.write_byte_data(current, 0x8A, self.address << 1)
            time.sleep(0.01)
        except Exception as e:
            log.warn(f"{self.name}: address programming failed - {e}")
            return
        if self._answers_at(self.address, self.address << 1):
            log.info(f"{self.name}: address programmed "
                     f"0x{current:02X} -> 0x{self.address:02X}")
            return
        # Attempt 2: clone variant applies the value raw — it now answers
        # at `address << 1`. Re-write the raw address from there.
        if self._answers_at(self.address << 1, self.address << 1):
            try:
                self._bus.write_byte_data(
                    self.address << 1, 0x8A, self.address)
                time.sleep(0.01)
            except Exception as e:
                log.warn(f"{self.name}: clone address programming failed - {e}")
                return
            if self._answers_at(self.address, self.address):
                log.info(f"{self.name}: address programmed (clone) "
                         f"0x{current:02X} -> 0x{self.address:02X}")
                return
            log.warn(f"{self.name}: clone chip not responding at "
                     f"0x{self.address:02X} after re-program")
        else:
            log.warn(f"{self.name}: chip not found at 0x{self.address:02X} "
                     f"or 0x{self.address << 1:02X} after write")

    def _start_measurement(self):
        """
        Start measurement according to the DETECTED silicon family.

        The module label cannot be trusted: the "VL53L1X" front module was
        identified as a genuine VL53L0X (0xC0=0xEE), and the module at the
        L0X address streamed live data with no trigger at all (VL53L1X
        behaviour). So:

          - "l0x": run the full Pololu continuous sequence (two prefix
            rounds — the second round is required and was the missing
            piece when the probe's single-round attempt stayed at 0).
          - "l1x": write SYSTEM__MODE_START (safety net; it measures on
            its own anyway).
          - "unknown": try the L0X sequence first, then the L1X start.

        Best-effort: failures are logged, never raised.
        """
        try:
            if self._family == "l1x":
                family.start_l1x(self._bus, self.address)
                log.info(f"{self.name}: VL53L1X silicon detected — "
                         f"L1X layout/trigger")
            else:
                family.start_l0x_continuous(self._bus, self.address)
                if self._family == "unknown":
                    try:
                        family.start_l1x(self._bus, self.address)
                    except Exception:
                        pass
                    log.warn(f"{self.name}: silicon ID unknown — "
                             f"applied both triggers")
                else:
                    log.info(f"{self.name}: VL53L0X silicon detected — "
                             f"ranging started (continuous)")
        except Exception as e:
            log.warn(f"{self.name}: start measurement failed - {e}")

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
        if not self._device:
            return None
        last_error = None
        for attempt in range(3):
            try:
                range_mm = family.read_range(
                    self._bus, self.address, self._family)
                if range_mm is None:
                    return 0.0
                return float(range_mm)
            except Exception as e:
                # Transient I2C errors (Errno 5 NACK, Errno 121 remote I/O)
                # are common when another sensor holds the bus; retry briefly.
                last_error = e
                time.sleep(0.005)
        self._log_error(f"read error - {last_error}")
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
        # Two-stage filter: first reject single-frame spikes (outlier), then
        # smooth remaining noise with moving average to prevent PID oscillation.
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
