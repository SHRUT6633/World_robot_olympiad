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
        # ψ̇ = (v/L)*tan(δ): kinematic yaw rate from no-slip bicycle model.
        # Zero when stationary because tan(δ) has no effect without forward motion.
        if abs(v) < 0.01:
            self.yaw_rate = 0.0
        else:
            self.yaw_rate = (v / wheelbase) * np.tan(delta)
        return self.yaw_rate

    def integrate_yaw(self, yaw):
        # Euler integration of yaw: ψ₊₁ = ψ + ψ̇*dt.
        # Assumes constant yaw_rate over the interval — accurate only when dt
        # is small relative to the yaw dynamics time constant.
        return yaw + self.yaw_rate * self.dt
