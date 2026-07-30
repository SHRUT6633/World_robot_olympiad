# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/fusion/complementary.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Complementary filter for IMU orientation
# =============================================================================

import numpy as np


class ComplementaryFilter:
    # Fuses accelerometer and gyroscope data to estimate pitch, roll, and yaw.
    #
    # Gyroscope provides good short-term precision (no external forces needed) but drifts
    # over time due to integration of bias. Accelerometer (for pitch/roll) and magnetometer
    # (for yaw) provide drift-free absolute references but are noisy and responsive to motion.
    #
    # The complementary filter combines the two with a blending factor alpha:
    #   angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
    #
    # alpha close to 1.0 = heavily trust gyro (smooth but can drift)
    # alpha close to 0.0 = heavily trust accelerometer (noisy but drift-free)
    # Typical values: 0.98–0.995

    def __init__(self, alpha=0.98, dt=0.01):
        self.alpha = alpha   # Blending factor (0–1)
        self.dt = dt         # Time step in seconds
        self.pitch = 0.0     # Current pitch estimate (radians)
        self.roll = 0.0      # Current roll estimate (radians)
        self.yaw = 0.0       # Current yaw estimate (radians)

    def update(self, accel, gyro, mag_heading=None):
        #
        # accel: 3-element [ax, ay, az] from accelerometer (m/s²)
        # gyro:  3-element [gx, gy, gz] angular velocity (rad/s)
        # mag_heading: optional absolute heading from magnetometer (radians)
        #
        # Returns (pitch, roll, yaw).
        #

        # Compute pitch and roll from accelerometer alone.
        # These formulas are valid when the robot is not accelerating linearly.
        # arctan2 handles signs correctly for each quadrant.
        accel_pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        accel_roll = np.arctan2(accel[1], accel[2])

        # Complementary filter: blend gyro integration with accelerometer reading
        self.pitch = self.alpha * (self.pitch + gyro[0] * self.dt) + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * (self.roll + gyro[1] * self.dt) + (1 - self.alpha) * accel_roll

        if mag_heading is not None:
            # If magnetometer heading is available, blend it with gyro yaw
            gyro_yaw = self.yaw + gyro[2] * self.dt
            self.yaw = self.alpha * gyro_yaw + (1 - self.alpha) * mag_heading
        else:
            # No absolute yaw reference — pure gyro integration (will drift over time)
            self.yaw += gyro[2] * self.dt

        return self.pitch, self.roll, self.yaw
