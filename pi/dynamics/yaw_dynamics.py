# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/dynamics/yaw_dynamics.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Yaw dynamics modelling using kinematic bicycle model
# =============================================================================

import numpy as np


class YawDynamics:
    # Models the yaw (heading) dynamics of the robot using the kinematic
    # bicycle model relationship: yaw_rate = (v / L) * tan(delta).
    #
    # This class maintains state (yaw_rate) and can integrate it to update
    # the absolute yaw angle. Useful for simulation and dead-reckoning when
    # gyroscope readings are unavailable or noisy.

    def __init__(self, dt=0.01):
        # dt: nominal time step (seconds) for integration.
        #   Used by integrate_yaw(). If the actual control loop runs at a
        #   different rate, pass the correct dt or it will accumulate error.
        self.dt = dt
        # yaw_rate: the current yaw rate (rad/s), maintained as internal state.
        # Initially zero (robot not turning).
        self.yaw_rate = 0.0

    def update(self, v, delta, wheelbase=0.26):
        # v: longitudinal velocity (m/s).
        # delta: steering angle (rad).
        # wheelbase: distance between axles (m). Default 0.26.
        # Returns: the computed yaw rate (rad/s), also stored in self.yaw_rate.
        #
        # Kinematic formula: psi_dot = (v / L) * tan(delta).
        # If v is near 0 (stationary), the robot cannot yaw — set to 0.
        #
        # Physical interpretation:
        #   - At low speed and same steering angle, yaw_rate is small.
        #   - At high speed, even small steering produces significant yaw_rate.
        # Changing wheelbase L changes the ratio: longer wheelbase = less
        # yaw for the same speed and steering (more stable at speed).
        if abs(v) < 0.01:
            self.yaw_rate = 0.0
        else:
            self.yaw_rate = (v / wheelbase) * np.tan(delta)
        return self.yaw_rate

    def integrate_yaw(self, yaw):
        # yaw: current absolute yaw angle (rad).
        # Returns: updated yaw angle after one time step (self.dt).
        #
        # Euler integration: yaw_new = yaw + yaw_rate * dt.
        # This assumes constant yaw_rate over the interval.
        # If dt is too large relative to how fast yaw_rate changes, the
        # integration accumulates error (can be significant in sharp turns).
        return yaw + self.yaw_rate * self.dt
