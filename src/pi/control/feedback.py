# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/feedback.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: PD feedback controller for steering heading correction
# =============================================================================

import numpy as np


class FeedbackSteering:
    # A proportional-derivative (PD) feedback controller for steering.
    # Corrects the robot's heading by reacting to the error between the
    # desired heading and the actual heading.
    # No integral term — relies on feedforward or other external compensation
    # for steady-state error.

    def __init__(self, kp=1.0, kd=0.1):
        # kp: proportional gain — immediate reaction to the current error.
        # kd: derivative gain — dampens oscillations by reacting to the
        #     rate of change of the error.
        # Increasing kp makes the response faster but can cause overshoot.
        # Increasing kd adds damping, reducing overshoot but may amplify
        # noise if too large.
        self.kp = kp
        self.kd = kd
        # _last_error stores the previous heading error for derivative computation.
        self._last_error = 0.0

    def compute(self, heading_error):
        # heading_error: the difference between the desired and actual heading
        #                (in radians, typically).
        # Returns: steering command = kp * error + kd * (error - last_error).
        #
        # The derivative term (heading_error - self._last_error) approximates
        # de/dt via finite difference. This predicts future error and dampens
        # the response.
        #
        # If kp is too large, the robot may oscillate around the target heading.
        # If kd is too large, steering becomes jerky/sensitive to noise.
        # Finite-difference derivative: de = heading_error_k - heading_error_{k-1}
        # Implicit sample period of 1 (steps) — dt scaling is handled externally
        derivative = heading_error - self._last_error
        self._last_error = heading_error
        # PD law: proportional reacts to current error, derivative predicts future error
        # No integral term — steady-state heading error is handled by feedforward or
        # the outer path-tracking controller (e.g., Stanley or MPC)
        return self.kp * heading_error + self.kd * derivative
