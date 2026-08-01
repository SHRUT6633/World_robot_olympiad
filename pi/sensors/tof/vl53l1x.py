# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/tof/vl53l1x.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# VL53L1X long-range Time-of-Flight sensor driver
# =============================================================================

import time
from ..base import SensorBase, FilteredSensorMixin
from ...system.logger import log
from . import xshut_manager


class VL53L1X(SensorBase, FilteredSensorMixin):
    """
    Driver for the STMicroelectronics VL53L1X long-range time-of-flight sensor.

    Physical meaning:
      Like the VL53L0X, the VL53L1X uses VCSEL laser pulses and measures
      time-of-flight. However, the VL53L1X has a longer range (up to 4 m)
      and better ambient-light rejection thanks to:
        - Multi-zone ranging (up to 4 × 4 = 16 zones).
        - Programmable timing budget (controls SNR vs. speed trade-off).
        - Improved SPAD (single-photon avalanche diode) array.

    I2C details:
      Default address: 0x29 (same as VL53L0X!).
      Address programming register: 0x8A.
      If both sensors are on the same bus, one MUST be re-addressed via XSHUT.

    Register map (key registers for distance reading):
      0x00–0x10 : Result registers (17 bytes).
        byte 14 (MSB) and byte 15 (LSB) = 16-bit range in millimetres.
        byte 16 = range status (0 = valid, 4 = signal too low, etc.).

    Differences from VL53L0X:
      1. Longer range: up to 4 m (vs. 1.2 m).
      2. Larger block read: 17 bytes (vs. 12).
      3. Status byte: must check byte 16 before trusting the reading.
      4. Timing budget: longer budget = better SNR but slower update rate.
         Default is ~50 ms (20 Hz). For fast robot motion you may need
         to reduce this via the ST API (not yet implemented here — the
         sensor runs at default firmware settings).

    Configuration parameters:
      bus       : I2C bus (1 on RPi).
      address   : I2C address (default 0x32 in this config).
      xshut_pin : GPIO pin for XSHUT hardware addressing.

    How the robot uses this:
      The VL53L1X is used for longer-range obstacle detection (1–4 m).
      Typical placements:
        - Forward-facing: detect obstacles at higher speed before braking.
        - Downward-facing: measure floor distance for altitude hold / drop-off
          detection (e.g., table edge).
      The status byte prevents the robot from reacting to invalid measurements
      (e.g. when the target is outside the sensor's range).
    """

    def __init__(self, name, bus=1, address=0x32, xshut_pin=None):
        """
        name      : Unique sensor name (e.g. "ToF_long_front").
        bus       : I2C bus number.
        address   : Desired I2C address after reprogramming.
        xshut_pin : BCM GPIO for XSHUT control (optional).
        """
        SensorBase.__init__(self, name)
        FilteredSensorMixin.__init__(self, window_size=5)
        self.bus = bus
        self.address = address
        self.xshut_pin = xshut_pin
        self._xshut = None
        self._device = None
        # Register this XSHUT pin NOW (at construction). All sensors exist
        # before init_all() runs, so when THIS sensor programs its address,
        # every other sensor's pin is already known and can be held in
        # hardware reset — otherwise the un-initialised sensors would still
        # be powered at 0x29 and receive the address write too.
        xshut_manager.register(xshut_pin)

    def init(self):
        """
        Initialise the VL53L1X sensor.

        Same sequence as VL53L0X:
          1. Open I2C bus.
          2. Pulse XSHUT and program address if xshut_pin given.
          3. Fall back to mock mode if hardware unavailable.

        Note: This does NOT configure the timing budget or inter-measurement
        period. The sensor runs with factory defaults (~50 ms timing budget,
        giving 20 Hz update rate). To change these, additional register writes
        via the ST VL53L1X API would be needed.
        """
        try:
            import smbus2
        except ImportError:
            log.warn(f"{self.name}: smbus2 not available, using mock")
            self._device = "mock"
            return
        try:
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
            self._start_measurement()
            log.info(f"{self.name}: VL53L1X @ 0x{self.address:02X} initialized")
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
        if got in (self.address << 1, self.address):
            return True
        log.warn(f"{self.name}: address reg 0x{got:02X}, "
                 f"expected 0x{self.address << 1:02X} or "
                 f"0x{self.address:02X}")
        return False

    def _find_boot_address(self):
        """
        Locate the sensor's current I2C address after power-up.

        Most VL53L1X modules boot at the factory default 0x29, but some
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
        Change sensor I2C address from its boot address to self.address.

        Register 0x8A holds the I2C slave address. Genuine ST chips encode
        it shifted (value = address << 1); some clone modules apply the
        written value raw as the 7-bit address. Both variants are handled
        here (write shifted form first, then the raw form if needed). The
        chip's current boot address (0x29, or 0x2C on some clone modules)
        is located first so sensors booting at a non-standard address are
        still programmed.
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
        Start ranging with triggers for BOTH chip families.

        The module configured as VL53L1X may in fact be a VL53L0X-clone
        (the raw-address behaviour proves the two modules share firmware
        quirks). A freshly powered ToF does not measure by itself, so:

          1. Write the VL53L0X free-running continuous sequence — on a
             genuine VL53L1X these registers (0x80/0xFF/0x91) are
             reserved/read-only, so it is a safe no-op; on a clone it
             starts real measurement.
          2. Write the VL53L1X SYSTEM__MODE_START (0x20 = 0x01) — starts
             ranging on a genuine VL53L1X; on a VL53L0X-clone it only
             lowers the (unused) signal limit register.

        Best-effort: failures are logged, never raised.
        """
        try:
            self._bus.write_byte_data(self.address, 0x80, 0x01)
            self._bus.write_byte_data(self.address, 0xFF, 0x01)
            self._bus.write_byte_data(self.address, 0x00, 0x00)
            try:
                sv = self._bus.read_byte_data(self.address, 0x91)
            except Exception:
                sv = 0x00
            self._bus.write_byte_data(self.address, 0x91, sv)
            self._bus.write_byte_data(self.address, 0x00, 0x01)
            self._bus.write_byte_data(self.address, 0xFF, 0x00)
            self._bus.write_byte_data(self.address, 0x80, 0x00)
            self._bus.write_byte_data(self.address, 0x00, 0x02)
            self._bus.write_byte_data(self.address, 0x20, 0x01)
            time.sleep(0.05)
            log.info(f"{self.name}: ranging started (dual trigger)")
        except Exception as e:
            log.warn(f"{self.name}: start measurement failed - {e}")

    def read_raw(self):
        """
        Read raw range + status from the sensor.

        Protocol:
          1. Block read 17 bytes from register 0x00.
          2. Bytes 14-15: 16-bit range in millimetres (VL53L1X layout).
             Bytes 10-11: 16-bit range in millimetres (VL53L0X layout —
             used when the 'VL53L1X' module is actually a VL53L0X clone).
          3. Byte 16: range status (VL53L1X layout); 0 = valid.

        The status check is critical: unlike the VL53L0X, the VL53L1X
        will return 0 or a garbage value when the target is out of range.
        We return None if no layout yields a valid reading, so the
        control loop does not act on invalid data.

        Mock mode: returns random values in [100, 3000] mm.
        """
        if self._device == "mock":
            import random
            return random.uniform(100, 3000)
        if not self._device:
            return None
        last_error = None
        for attempt in range(3):
            try:
                # Block read 17 bytes from result register 0x00; bytes 14-15
                # hold the 16-bit range (mm) on a VL53L1X, bytes 10-11 on a
                # VL53L0X, byte 16 holds the range status on a VL53L1X.
                data = self._bus.read_i2c_block_data(self.address, 0x00, 17)
                l1x_mm = (data[14] << 8) | data[15]
                l0x_mm = (data[10] << 8) | data[11]
                status = data[16]
                if l1x_mm > 0 and status == 0:
                    return float(l1x_mm)
                if l0x_mm > 0:
                    return float(l0x_mm)
                return None
            except Exception as e:
                # Transient I2C errors (Errno 5 NACK, Errno 121 remote I/O)
                # are common when another sensor holds the bus; retry briefly.
                last_error = e
                time.sleep(0.005)
        self._log_error(f"read error - {last_error}")
        return None
        last_error = None
        for attempt in range(3):
            try:
                # Block read 17 bytes from result register 0x00; bytes 14-15
                # hold the 16-bit range (mm), byte 16 holds the range status.
                data = self._bus.read_i2c_block_data(self.address, 0x00, 17)
                range_mm = (data[14] << 8) | data[15]
                status = data[16]
                # status=0 means valid measurement; non-zero means signal too low,
                # sigma too high, or wrap-around — discard to avoid acting on garbage.
                if status == 0:
                    return float(range_mm)
                return None
            except Exception as e:
                # Transient I2C errors (Errno 5 NACK, Errno 121 remote I/O)
                # are common when another sensor holds the bus; retry briefly.
                last_error = e
                time.sleep(0.005)
        self._log_error(f"read error - {last_error}")
        return None

    def read(self):
        """
        Filtered read: outlier rejection → moving average.

        Same two-stage filter as VL53L0X. The VL53L1X is inherently less
        noisy than the VL53L0X at long range, but filtering still helps
        when the target surface has low reflectivity (e.g. black carpet).
        """
        raw = super().read()
        if raw is None:
            return None
        # Same two-stage filtering as VL53L0X: outlier rejection protects against
        # spurious readings from low-reflectivity surfaces at long range.
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
