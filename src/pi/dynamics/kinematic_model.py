# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/dynamics/kinematic_model.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Kinematic bicycle model for 4WS forward simulation
# =============================================================================

import numpy as np
from .steering_modes import SteeringMode, compute_4ws_angles


class KinematicModel:
    def __init__(self, wheelbase=0.26, steering_mode=SteeringMode.SAME_PHASE):
        self.L = wheelbase
        self.steering_mode = steering_mode

    def set_steering_mode(self, mode: SteeringMode):
        self.steering_mode = mode

    def update(self, x, y, heading, v, steering_input, dt):
        front_angle, rear_angle, radius = compute_4ws_angles(
            steering_input, self.steering_mode, max_steering_rad=np.radians(30)
        )
        # 4WS effective delta = δf - δr: same-phase reduces curvature,
        # opposite-phase doubles it, crab-walk zeroes it out.
        effective_delta = front_angle - rear_angle
        # Kinematic bicycle: position integrates heading*velocity,
        # heading rate = (v/L)*tan(δeff) from no-slip constraint.
        x_next = x + v * np.cos(heading) * dt
        y_next = y + v * np.sin(heading) * dt
        heading_next = heading + (v / self.L) * np.tan(effective_delta) * dt
        return x_next, y_next, heading_next

    def compute_steering(self, v, yaw_rate):
        # Inverse bicycle model: given a desired yaw_rate, compute the
        # steering angle needed: δ = arctan(L * ψ̇ / v).
        if abs(v) < 0.01:
            return 0.0
        return np.arctan(self.L * yaw_rate / v)
