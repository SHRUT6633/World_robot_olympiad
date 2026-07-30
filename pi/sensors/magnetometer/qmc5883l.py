# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/magnetometer/qmc5883l.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# QMC5883L 3-axis magnetometer driver
# =============================================================================

import time
import numpy as np
from ..base import SensorBase
from ...system.logger import log


class QMC5883L(SensorBase):
    """
    Driver for the QMC5883L 3-axis magnetometer (compass).

    Physical meaning:
      The QMC5883L measures the strength and direction of the magnetic
      field along three perpendicular axes (X, Y, Z). It uses
      magneto-resistive sensors arranged in a Wheatstone bridge.

      The Earth's magnetic field (≈ 25–65 µT depending on location) is
      the reference. The sensor reads this field vector, from which we
      compute the heading (compass bearing) relative to magnetic north.

      In a robot, the magnetometer is used for:
        - Absolute heading reference (unlike the gyro, it does not drift).
        - Combined with gyro+accel for drift-free yaw estimation.

    I2C details:
      Address: 0x0D (QMC5883L) or 0x1E (HMC5883L — different chip).
      This driver uses 0x0D for the QMC5883L.

    Register map (key registers):
      0x00–0x05 : Data output registers (6 bytes, X_L, X_H, Y_L, Y_H, Z_L, Z_H).
      0x09      : Control Register 1 (mode, ODR, range, oversampling).
      0x0B      : Control Register 2 (soft reset, INT enable).
      0x0D      : Status register (DRDY: data ready flag).

    Configuration (register 0x09 = 0x1D):
      Bits [7:6] = 00 : Mode = Continuous measurement.
      Bits [5:4] = 01 : ODR = 50 Hz (output data rate).
      Bits [3:2] = 00 : RNG = ±2 gauss (measurement range).
      Bits [1:0] = 01 : OSR = 64 (oversampling ratio).

    Register 0x0B = 0x01: Soft reset (clears all registers to defaults)
                          then must re-write 0x09.

    Raw data format:
      Each axis is stored in two consecutive registers (little-endian:
      data[0] = X_L, data[1] = X_H).
      Values are 16-bit signed.

    Hard-iron calibration:
      Permanent magnetic fields from the robot's motors, battery, and
      wiring create a constant offset (hard-iron distortion).
      calibrate_hard_iron() finds the max/min of each axis while rotating
      the robot 360°, then hard_iron = (max + min) / 2.

    Soft-iron calibration:
      Ferrous materials (iron, steel) in the environment distort the
      field non-uniformly (soft-iron distortion). This is modelled as
      a 3×3 matrix. The current implementation just sets it to identity
      (no correction). A proper calibration would use ellipsoid fitting.

    Heading computation:
      heading(°, with tilt compensation):
        Uses pitch and roll from the accelerometer to project the magnetic
        field onto the horizontal plane before computing arctan2(y, x).
        Without tilt compensation, the heading is only accurate when the
        robot is perfectly level (±2° pitch/roll can cause ±10° heading
        error).

    Configuration parameters:
      bus     : I2C bus (1 on RPi).
      address : I2C address (0x0D).

    How the robot uses this:
      - After calibration, read() returns a hard-iron compensated vector.
      - heading(mag, accel) returns the tilt-compensated heading in degrees.
      - The navigation system uses this as the absolute orientation reference,
        correcting gyro yaw drift via a complementary filter or EKF.
    """

    def __init__(self, bus=1, address=0x0D):
        super().__init__("QMC5883L")
        self.bus = bus
        self.address = address
        self._device = None
        # H: constant hard-iron offset vector (bias from the robot's own
        # magnetic field — motors, battery, wiring). Subtracted from raw.
        self.hard_iron = np.zeros(3)
        # S: 3×3 soft-iron scaling matrix correcting for ferrous material
        # distortion in the environment. Identity = no correction applied.
        self.soft_iron = np.eye(3)

    def init(self):
        """
        Initialise the QMC5883L magnetometer.

        Register writes:
          1. 0x09 ← 0x1D : Continuous mode, 50 Hz, ±2 gauss, 64× oversampling.
          2. 0x0B ← 0x01 : Soft reset (clears previous settings).
          3. 0x09 ← 0x1D : Re-write config after soft reset (register 0x0B
             resets 0x09 to defaults).

        The ±2 gauss range is appropriate for the Earth's field (~0.5 gauss).
        For robots near strong magnets (speakers, motors), ±8 gauss may be
        needed (set RNG bits accordingly).

        Oversampling (OSR = 64): Averages 64 internal measurements per output.
        Higher OSR = lower noise but slower max ODR. 64× at 50 Hz is good.
        """
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus)
            # 0x09 = Control Reg 1: 0x1D = continuous mode, 50 Hz ODR,
            # ±2 G range, 64× oversampling (averages 64 samples per output).
            self._bus.write_byte_data(self.address, 0x09, 0x1D)
            # 0x0B = Control Reg 2: 0x01 = soft reset (clears all registers).
            # Must re-write 0x09 after this because reset defaults 0x09 to 0x00.
            self._bus.write_byte_data(self.address, 0x0B, 0x01)
            self._bus.write_byte_data(self.address, 0x09, 0x1D)
            self._device = True
            log.info("QMC5883L initialized")
        except ImportError:
            log.warn("QMC5883L: smbus2 not available, using mock")
            self._device = "mock"

    def read_raw(self):
        """
        Read raw magnetic field vector from the sensor.

        Reads 6 bytes from register 0x00:
          Byte 0 = X_LSB, Byte 1 = X_MSB
          Byte 2 = Y_LSB, Byte 3 = Y_MSB
          Byte 4 = Z_LSB, Byte 5 = Z_MSB

        Each axis: value = (MSB << 8) | LSB (little-endian).
        Convert to signed 16-bit (two's complement).

        Mock mode: returns random values in [-300, 300] for each axis.

        Returns:
          numpy array (3,) — raw magnetic field in sensor counts (not gauss).
          To convert to gauss: divide by the sensitivity (e.g. 2048 LSB/gauss
          at ±2 gauss range).

        Note: Always read all 6 bytes in one block read for data coherency
        (the sensor updates all axes simultaneously).
        """
        if self._device == "mock":
            import random
            return np.array([random.uniform(-300, 300) for _ in range(3)])
        try:
            # Block read 6 bytes from register 0x00 (X_L, X_H, Y_L, Y_H, Z_L, Z_H).
            data = self._bus.read_i2c_block_data(self.address, 0x00, 6)
            # Data is little-endian: LSB first, MSB second.
            x = (data[1] << 8) | data[0]
            y = (data[3] << 8) | data[2]
            z = (data[5] << 8) | data[4]
            # Convert unsigned 16-bit to signed two's complement.
            x = x - 65536 if x > 32767 else x
            y = y - 65536 if y > 32767 else y
            z = z - 65536 if z > 32767 else z
            return np.array([x, y, z], dtype=float)
        except Exception as e:
            self._log_error(str(e))
            return np.zeros(3)

    def read(self):
        """
        High-level read with hard-iron and soft-iron compensation.

        Compensated = S * (raw - H)

        Where:
          S = soft_iron matrix (3×3).
          H = hard_iron offset (3,).

        After compensation, the field vector has zero offset and is
        spherically normalised (if soft_iron is correct).

        Returns:
          numpy array (3,) — compensated magnetic field, or None if
          the sensor is disabled.
        """
        raw = super().read()
        if raw is None:
            return None
        # Apply hard-iron subtraction then soft-iron matrix multiplication:
        # B_compensated = S · (B_raw - H) — centres the ellipsoid then
        # maps it onto a sphere for distortion-free heading.
        compensated = self.soft_iron @ (raw - self.hard_iron)
        return compensated

    def calibrate_hard_iron(self, samples=500):
        """
        Estimate hard-iron offsets by rotating the robot 360°.

        The locus of raw readings from a 360° rotation in the horizontal
        plane is a circle (or ellipsoid) offset from the origin by the
        hard-iron vector.

        Algorithm:
          1. Collect `samples` readings while slowly rotating the robot.
          2. Track min and max of each axis.
          3. hard_iron = (max + min) / 2 for each axis.

        For best results:
          - Rotate the robot slowly (full rotation in ~30 s).
          - Tilt the robot to all orientations if calibrating in 3D.
          - Keep away from large metal objects (tables with steel frames).

        After calibration, the compensated readings should be centred
        on zero when the magnetic field is removed (theoretically).
        """
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
        """
        Calibrate soft-iron distortion.

        Currently a no-op that sets soft_iron to identity.

        A proper implementation would:
          1. Collect readings from many orientations.
          2. Fit an ellipsoid to the data (least-squares).
          3. Compute the S matrix that maps the ellipsoid to a sphere.
          4. Store S as self.soft_iron.

        Without soft-iron calibration, the heading accuracy may be
        degraded by ±5–10° in environments with ferrous materials.
        """
        self.soft_iron = np.eye(3)

    def heading(self, mag, accel=None):
        """
        Compute the heading (bearing) from magnetic field vector.

        mag   : numpy array (3,) — compensated magnetic field.
        accel : numpy array (3,) — acceleration from IMU (for tilt
                compensation). If None, horizontal-plane heading only.

        Without tilt compensation (accel=None):
          heading = arctan2(y, x)  [simple 2D compass]

        With tilt compensation:
          Projects the magnetic field onto the horizontal plane using
          pitch and roll from the accelerometer:
            pitch = arctan2(-ax, sqrt(ay² + az²))
            roll  = arctan2(ay, az)
          Then rotates mag by pitch and roll to get horizontal components.

        Returns:
          heading in degrees (0° = North, 90° = East, 180° = South, 270° = West).

        Why tilt compensation is important:
          The Earth's field has a vertical component (varies by latitude).
          If the robot tilts, the X and Y axes sense part of the vertical
          field, causing large heading errors. Tilt compensation removes
          this effect.
        """
        if accel is not None:
            # Compute pitch and roll from accelerometer (gravity vector).
            pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
            roll = np.arctan2(accel[1], accel[2])
            # Tilt-compensated horizontal components.
            x = mag[0] * np.cos(pitch) + mag[2] * np.sin(pitch)
            y = mag[0] * np.sin(roll) * np.sin(pitch) + mag[1] * np.cos(roll) - mag[2] * np.sin(roll) * np.cos(pitch)
        else:
            # Simple 2D compass (valid only when robot is level).
            x, y = mag[0], mag[1]
        return np.degrees(np.arctan2(y, x))

    def close(self):
        if hasattr(self, "_bus") and self._bus:
            try:
                # Release the I2C bus so magnetometer is free for other
                # processes or re-initialisation on the next power cycle.
                self._bus.close()
            except Exception:
                pass
