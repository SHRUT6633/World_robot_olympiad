# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/feedforward.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Feedforward steering controller based on kinematic bicycle model
# =============================================================================

import numpy as np


class FeedforwardSteering:
    # A feedforward steering controller based on the kinematic bicycle model.
    # For a given path curvature, it computes the steering angle required to
    # follow that curvature, assuming no slip (pure rolling).
    #
    # Steering angle = arctan(L * curvature)
    # where L is the wheelbase and curvature = 1 / turning_radius.
    #
    # This is the inverse of the Ackermann kinematic model.

    def __init__(self, wheelbase=0.26):
        # L: distance between front and rear axles (meters).
        #    0.26 m is typical for a small robot car (e.g., 1:10 scale).
        #    Increasing L means a larger steering angle is needed for the
        #    same curvature (less responsive steering).
        self.L = wheelbase

    def compute(self, curvature, v=None):
        # curvature: 1 / turning radius of the desired path (1/m).
        # v: current velocity (m/s) — accepted but NOT used here.
        #    The kinematic model is velocity-independent; it assumes
        #    instantaneous steady-state cornering.
        # Returns: steering angle (radians).
        #
        # If curvature is near zero (straight line), return 0 (no steering).
        # The 1e-6 threshold prevents division by zero / numerical issues.
        #
        # Changing L directly scales the relationship: a longer wheelbase
        # means you need more steering angle to achieve the same curvature.
        if abs(curvature) < 1e-6:
            return 0.0
        return np.arctan(self.L * curvature)
