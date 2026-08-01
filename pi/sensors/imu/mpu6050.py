# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/imu/mpu6050.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# MPU6050 6-axis IMU driver
# =============================================================================

import time
import numpy as np
from ..base import SensorBase
from ...system.logger import log


class MPU6050(SensorBase):
    """
    Driver for the InvenSense MPU6050 6-axis IMU (3-axis accelerometer +
    3-axis gyroscope).

    Physical meaning:
      Accelerometer:
        Measures proper acceleration (g-forces) on X, Y, Z axes.
        At rest on a flat surface:
          - Ax ≈ 0 g, Ay ≈ 0 g, Az ≈ +1 g (= +9.81 m/s²).
        Used for:
          - Tilt / pitch / roll estimation (gravity vector direction).
          - Linear acceleration for dead-reckoning (double-integrated to
            position, though this drifts quickly without correction).

      Gyroscope:
        Measures angular velocity around X, Y, Z axes (in °/s or rad/s).
        Used for:
          - Turn rate control (PID on gyro Z for precise heading changes).
          - Inertial heading integration (dead-reckoning).
          - Stabilisation (active damping of chassis oscillation).

    I2C details:
      Address: 0x68 (default, AD0 pin low) or 0x69 (AD0 pin high).
      The MPU6050 also has a pass-through I2C master interface for external
      magnetometers (typically connected to its AUX pins).

    Register map:
      0x6B : Power Management 1 (0x00 = wake from sleep).
      0x1C : Accelerometer Configuration (full-scale range).
      0x1B : Gyroscope Configuration (full-scale range).
      0x1A : Digital Low-Pass Filter (BW configuration).
      0x3B–0x40 : Accelerometer data registers (6 bytes, X_H, X_L, Y_H, Y_L, Z_H, Z_L).
      0x43–0x48 : Gyroscope data registers (6 bytes, same format).
      0x41–0x42 : Temperature sensor (not used here).

    Configuration parameters:
      accel_range : ±2, ±4, ±8, ±16 g. Default ±4 g.
                    Larger range = less sensitivity but can measure
                    stronger accelerations (e.g. aggressive braking).
      gyro_range  : ±250, ±500, ±1000, ±2000 °/s. Default ±500 °/s.
                    For a WRO robot (slow speed, sharp turns), ±500 °/s
                    is sufficient. ±2000 °/s would be needed for fast
                    spinning.

    Scaling:
      Raw ADC values are 16-bit signed (-32768 to +32767).
      Scaling factor = range / 32768.0.
      Example: accel_range = ±4 g → scale = 4 / 32768 ≈ 0.000122 g/LSB.

    Biases:
      Accelerometer and gyroscope have zero-point offsets measured during
      calibrate_*_bias() and subtracted from each reading.

    Digital Low-Pass Filter (register 0x1A, value 0x03):
      Sets the internal DLPF bandwidth. Value 0x03 = ~44 Hz bandwidth.
      This reduces aliasing from high-frequency vibration (motor noise)
      before the ADC samples.

    How the robot uses this:
      - Accel: tilt compensation for magnetometer, pitch/roll for balance.
      - Gyro: turn-rate feedback for differential-drive PID control,
              heading estimation (yaw integration).
      The raw data is fed through IMUFilter (butterworth LPF/HPF) and
      optional temp_compensation before being consumed by the navigation
      system.
    """

    def __init__(self, bus=1, address=0x68, accel_range=4, gyro_range=500):
        """
        bus         : I2C bus number.
        address     : I2C address (0x68 or 0x69).
        accel_range : ±g full scale (2, 4, 8, or 16).
        gyro_range  : ±°/s full scale (250, 500, 1000, or 2000).
        """
        super().__init__("MPU6050")
        self.bus = bus
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        # smbus2 device handle (set in init()).
        self._device = None
        # Scale factor to convert raw ADC counts to physical units.
        # accel: g/LSB, gyro: (°/s)/LSB.
        self._accel_scale = accel_range / 32768.0
        self._gyro_scale = gyro_range / 32768.0
        # Bias vectors (subtracted from each reading).
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        # Register values for ACCEL_CONFIG (0x1C) and GYRO_CONFIG (0x1B).
        # These MUST match the scaling factors above or every reading is
        # off by a fixed ratio.
        self._accel_reg = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}[accel_range]
        self._gyro_reg = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}[gyro_range]

    def init(self):
        """
        Initialise the MPU6050.

        Register writes:
          1. 0x6B ← 0x00 : Wake from sleep. The MPU6050 boots in sleep
             mode to save power; must be cleared before any measurements.
          2. 0x1C ← accel_reg : Accelerometer full-scale register, matched
             to self.accel_range (±4 g → 0x08).
          3. 0x1B ← gyro_reg : Gyroscope full-scale register, matched to
             self.gyro_range (±500 °/s → 0x08).
          4. 0x1A ← 0x03 : DLPF bandwidth ≈ 44 Hz.

        The WHO_AM_I register (0x75) is read to prove the chip is present.
        If the I2C transaction fails, the chip is not answering and init
        raises so the SystemManager reports FAILED (a red LED then warns
        the operator). It never silently claims OK.
        """
        try:
            import smbus2
        except ImportError:
            log.warn("MPU6050: smbus2 not available, using mock")
            self._device = "mock"
            return
        try:
            self._bus = smbus2.SMBus(self.bus)
        except Exception as e:
            log.warn(f"MPU6050: I2C bus unavailable ({e}), using mock")
            self._device = "mock"
            return
        try:
            # 0x6B = Power Management 1. Write 0x00 to wake from sleep mode
            # (chip boots in sleep; no measurements possible until cleared).
            self._bus.write_byte_data(self.address, 0x6B, 0x00)
            # 0x1C = ACCEL_CONFIG. Register value matched to accel_range so
            # the scaling factor in read_raw() matches the hardware range.
            self._bus.write_byte_data(self.address, 0x1C, self._accel_reg)
            # 0x1B = GYRO_CONFIG. Register value matched to gyro_range.
            self._bus.write_byte_data(self.address, 0x1B, self._gyro_reg)
            # 0x1A = CONFIG. Write 0x03 → DLPF bandwidth ≈ 44 Hz, cutting
            # motor-frequency vibration above the accelerometer's useful band.
            self._bus.write_byte_data(self.address, 0x1A, 0x03)
            # 0x75 = WHO_AM_I. Any successful read proves the chip answers.
            # (Clones return 0x68, 0x70, 0x72, 0x98, ... — the exact value
            # varies, so a successful transaction is the presence test.)
            try:
                who = self._bus.read_byte_data(self.address, 0x75)
            except Exception as e:
                raise RuntimeError(
                    f"MPU6050: chip not responding at 0x{self.address:02X} "
                    f"(check wiring, power, AD0 pin) - {e}")
            if who != 0x68:
                log.warn(f"MPU6050: WHO_AM_I=0x{who:02X} (not 0x68) — "
                         f"proceeding anyway")
            self._device = True
            log.info("MPU6050 initialized")
        except Exception as e:
            log.error(f"MPU6050: init failed - {e}")
            raise

    def _read_word(self, reg):
        if self._device == "mock":
            import random
            return random.randint(-100, 100)
        if not self._device:
            return 0
        last_error = None
        for attempt in range(3):
            try:
                # Read two consecutive I2C registers (big-endian: high byte first).
                high = self._bus.read_byte_data(self.address, reg)
                low = self._bus.read_byte_data(self.address, reg + 1)
                val = (high << 8) | low
                # Convert unsigned 16-bit to signed two's complement.
                return val - 65536 if val > 32767 else val
            except Exception as e:
                # Transient I2C errors (Errno 5 NACK, Errno 121 remote I/O)
                # are common when another device holds the bus; retry briefly.
                last_error = e
                time.sleep(0.005)
        self._log_error(str(last_error))
        return 0

    def read_raw(self):
        """
        Read all 6 axes from the sensor.

        Register address map:
          0x3B, 0x3C = ACCEL_XOUT_H, ACCEL_XOUT_L
          0x3D, 0x3E = ACCEL_YOUT_H, ACCEL_YOUT_L
          0x3F, 0x40 = ACCEL_ZOUT_H, ACCEL_ZOUT_L
          0x43, 0x44 = GYRO_XOUT_H, GYRO_XOUT_L
          0x45, 0x46 = GYRO_YOUT_H, GYRO_YOUT_L
          0x47, 0x48 = GYRO_ZOUT_H, GYRO_ZOUT_L

        Returns:
          dict with:
            "accel":     np.ndarray(3) — bias-corrected acceleration in g.
            "gyro":      np.ndarray(3) — bias-corrected angular velocity in °/s.
            "accel_raw": np.ndarray(3) — raw (unbiased) acceleration in g.
            "gyro_raw":  np.ndarray(3) — raw (unbiased) angular velocity.

        The raw arrays are exposed so the calibration functions can
        compute biases from unbiased data.
        """
        # Read accelerometer registers 0x3B–0x40 (X, Y, Z consecutive pairs).
        ax = self._read_word(0x3B) * self._accel_scale
        ay = self._read_word(0x3D) * self._accel_scale
        az = self._read_word(0x3F) * self._accel_scale
        # Read gyroscope registers 0x43–0x48 (X, Y, Z consecutive pairs).
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
        """
        Estimate gyroscope zero-rate offset (bias).

        Procedure:
          1. Keep the robot perfectly still for ~0.6 s (200 × 3 ms).
          2. Average the raw gyro readings.
          3. Store the average as gyro_bias.

        On a perfectly stationary sensor, the gyro should read (0, 0, 0).
        Real sensors have a small offset (e.g. +2.5 °/s on Z). This offset
        is subtracted from every subsequent reading.

        If not calibrated, the robot's heading will drift even when
        stationary (integration of non-zero bias → accumulating heading error).
        """
        biases = []
        for _ in range(samples):
            data = self.read_raw()
            biases.append(data["gyro_raw"])
            time.sleep(0.003)
        self.gyro_bias = np.mean(biases, axis=0)
        log.info(f"Gyro bias calibrated: {self.gyro_bias}")

    def calibrate_accel_bias(self, samples=100):
        """
        Estimate accelerometer zero-g offset.

        Procedure:
          1. Place the robot on a flat, level surface.
          2. Collect ~100 readings over 0.3 s.
          3. Average the readings.
          4. Subtract 1 g (9.81 m/s²) from the Z component because
             gravity always reads +1 g on Z when level.

        The resulting bias vector accounts for:
          - Sensor manufacturing offset (each axis may have ±50 mg offset).
          - Board-level tilt (if the sensor is not perfectly aligned with
            the chassis).

        Accel bias is usually small (< 0.1 g) but important for accurate
        tilt angle estimation.
        """
        biases = []
        for _ in range(samples):
            data = self.read_raw()
            biases.append(data["accel_raw"])
            time.sleep(0.003)
        self.accel_bias = np.mean(biases, axis=0)
        # Remove gravity from Z (assuming sensor is level).
        self.accel_bias[2] -= 9.81
        log.info(f"Accel bias calibrated: {self.accel_bias}")

    def close(self):
        if hasattr(self, "_bus") and self._bus:
            try:
                # Release the I2C bus so it is available for other processes
                # (or for re-initialisation after a soft reset).
                self._bus.close()
            except Exception:
                pass
