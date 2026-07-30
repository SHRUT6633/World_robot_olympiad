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
    #
    # Why not a full Kalman filter? The CF is computationally lighter (no matrix ops),
    # requires no noise model tuning (only one parameter alpha), and performs well
    # when sensor noise characteristics are stationary. For WRO's IMU, where low-cost
    # MEMS sensors have well-understood noise floors, the CF is sufficient for
    # attitude estimation. A Kalman filter would only marginally improve accuracy
    # at much higher CPU cost on the Raspberry Pi.

    def __init__(self, alpha=0.98, dt=0.01):
        # alpha = 0.98 means 98% gyro, 2% accel/mag per update
        # Equivalent cut-off frequency: f_c = (1 - alpha) / (2 * pi * dt)
        # For alpha=0.98, dt=0.01: f_c ≈ 0.32 Hz — passes slow drift correction,
        # rejects high-frequency accel noise. Tuned empirically for WRO track
        # speeds (0.5-2.0 m/s) where body accelerations are modest.
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
        #
        # Pitch:  θ = atan2(-a_x, sqrt(a_y^2 + a_z^2))
        #   The negative sign aligns with the IMU frame: +pitch = nose up = -a_x
        # Roll:   φ = atan2(a_y, a_z)
        #   When level: a_z ≈ g, a_y ≈ 0  → roll ≈ 0
        #   Tilting right: a_y positive  → roll positive
        #
        # Assumption: the only significant acceleration is gravity (no linear accel).
        # During WRO acceleration/braking, linear acceleration corrupts these estimates,
        # but the gyro integration in the CF blends out the transient error.
        accel_pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        accel_roll = np.arctan2(accel[1], accel[2])

        # Complementary filter: blend gyro integration with accelerometer reading
        # Continuous-time equivalent: angle_dot = gyro + Kp * (accel_angle - angle)
        # Discretised with forward Euler: coefficient alpha = 1 - Kp * dt
        # High-pass on gyro (removes drift), low-pass on accel (removes noise)
        self.pitch = self.alpha * (self.pitch + gyro[0] * self.dt) + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * (self.roll + gyro[1] * self.dt) + (1 - self.alpha) * accel_roll

        if mag_heading is not None:
            # If magnetometer heading is available, blend it with gyro yaw
            # Yaw has no gravity reference (yaw is rotation about vertical axis),
            # so we rely on the magnetometer (Earth's magnetic field) for absolute heading.
            # Same alpha blending: gyro yaw integration high-passed,
            # magnetometer heading low-passed to remove hard-iron noise.
            gyro_yaw = self.yaw + gyro[2] * self.dt
            self.yaw = self.alpha * gyro_yaw + (1 - self.alpha) * mag_heading
        else:
            # No absolute yaw reference — pure gyro integration (will drift over time)
            # Magnetometer can be temporarily unavailable (e.g., magnetic interference
            # from the track's metal structures), so we fall back to dead-reckoning yaw.
            self.yaw += gyro[2] * self.dt

        return self.pitch, self.roll, self.yaw
