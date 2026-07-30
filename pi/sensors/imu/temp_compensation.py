# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/imu/temp_compensation.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# MEMS IMU temperature compensation
# =============================================================================

import numpy as np
from ...system.logger import log


class IMUTempCompensation:
    """
    Temperature compensation for MEMS IMU bias drift.

    Physical background:
      MEMS accelerometers and gyroscopes are sensitive to temperature.
      As the sensor heats up (due to the robot's motors, battery, or
      ambient conditions), the bias offsets change.

      Typical drift per °C:
        - Accelerometer: ~0.002 g/°C (2 mg/°C).
        - Gyroscope:     ~0.02 °/s/°C (20 mdps/°C).

      Over a 10 °C temperature change (cold start → motor heat):
        - Accel bias shift: up to 0.02 g → tilt error of ~1.1°.
        - Gyro bias shift:  up to 0.2 °/s → heading drift of ~12°
          after 60 seconds of integration.

    This class implements a simple first-order linear compensation:
        compensated = raw - coeff * (T_current - T_reference)

    Where:
      - T_reference is the temperature at which the sensor was calibrated
        (default 25 °C).
      - coeff is the temperature coefficient (per axis, per sensor).
      - T_current is the current sensor temperature (from the MPU6050's
        internal temperature sensor, or an external ambient sensor).

    Current limitation:
      read_temp() returns a hardcoded 25.0 °C. To make this effective,
      it should read the MPU6050's temperature register (0x41–0x42) or
      an external thermistor.

    How the robot uses this:
      Before each control loop iteration, the IMU data is passed through
      compensate() to remove temperature-dependent bias drift. This
      significantly improves:
        - Heading hold accuracy during the match.
        - Dead-reckoning repeatability across matches (different ambient temps).
        - Tilt angle stability for vision-based line following.
    """

    def __init__(self):
        # Reference temperature (°C) at which the sensor was calibrated.
        # The bias values stored in MPU6050.accel_bias / .gyro_bias are
        # valid at this temperature.
        self.ref_temp = 25.0
        # Temperature coefficients for accelerometer (g/°C per axis).
        # Typical: ~0.002 g/°C for each of X, Y, Z.
        self.accel_temp_coeff = np.array([0.002, 0.002, 0.002])
        # Temperature coefficients for gyroscope (°/s/°C per axis).
        # Typical: ~0.02 °/s/°C for each of X, Y, Z.
        self.gyro_temp_coeff = np.array([0.02, 0.02, 0.02])

    def read_temp(self):
        """
        Read the current sensor temperature.

        Currently returns a hardcoded 25.0 °C placeholder.

        TODO: Read the MPU6050 internal temperature sensor.
        Registers:
          0x41 = TEMP_OUT_H (MSB)
          0x42 = TEMP_OUT_L (LSB)
        Conversion formula (from datasheet):
          T(°C) = (raw / 340.0) + 36.53
        """
        return 25.0

    def compensate(self, accel, gyro, temp=None):
        """
        Apply temperature compensation to IMU readings.

        accel : numpy array (3,) — acceleration in g (before bias subtraction).
        gyro  : numpy array (3,) — angular velocity in °/s (before bias subtraction).
        temp  : current temperature in °C. If None, uses read_temp().

        Returns:
          (accel_comp, gyro_comp) — temperature-compensated readings.

        Formula:
          accel_comp = accel - accel_temp_coeff * (T - T_ref)
          gyro_comp  = gyro  - gyro_temp_coeff  * (T - T_ref)

        Note: The signs are chosen so that a positive coefficient with
        increasing temperature reduces the reading (compensating for a
        positive bias shift). Adjust coefficients based on empirical testing.
        """
        if temp is None:
            temp = self.read_temp()
        dt = temp - self.ref_temp
        accel_comp = accel - self.accel_temp_coeff * dt
        gyro_comp = gyro - self.gyro_temp_coeff * dt
        return accel_comp, gyro_comp
