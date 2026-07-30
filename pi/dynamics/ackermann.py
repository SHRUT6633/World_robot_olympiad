# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/dynamics/ackermann.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Ackermann steering geometry for inner/outer wheel angles
# =============================================================================

import numpy as np


class AckermannGeometry:
    # Models the Ackermann steering geometry used in cars.
    # In a real vehicle, the inner and outer wheels turn at different angles
    # so they all follow circles about a common turn center (no tire scrubbing).
    #
    # This class computes:
    #   1. Inner/outer wheel angles for a given average steering angle.
    #   2. The turning radius from the bicycle-model perspective.

    def __init__(self, wheelbase=0.26, track_width=0.18):
        # L: distance between front and rear axles (m).
        # W: lateral distance between the two front wheels (track width) (m).
        # Changing W affects how different inner vs. outer angles are.
        # A wider track means a larger angle difference between inner/outer.
        self.L = wheelbase
        self.W = track_width

    def inner_outer_angles(self, delta, direction=1):
        # delta: the average (bicycle-model) steering angle (rad).
        # direction: +1 for left turn, -1 for right turn.
        #   This sign flips which wheel is "inner" vs "outer".
        # Returns: (delta_inner, delta_outer) in radians.
        #
        # R: turn radius of the rear axle centerline.
        #   Computed from the bicycle model: R = L / tan(|delta|).
        #   The 1e-6 prevents division by zero when delta = 0.
        # delta_inner: the wheel physically inside the turn (larger angle).
        #   R - W/2: shorter radius → larger steering angle needed.
        # delta_outer: the wheel outside the turn (smaller angle).
        #   R + W/2: longer radius → smaller steering angle.
        #
        # If W = 0 (zero track width), both angles are equal (delta).
        # If delta is large, the difference between inner/outer grows.
        # R = L/tan(|δ|): bicycle-model turn radius of the rear axle centreline.
        R = self.L / (np.tan(abs(delta)) + 1e-6)
        # Inner wheel follows a tighter arc (R - W/2), requiring a larger angle.
        # Outer wheel follows a wider arc (R + W/2), requiring a smaller angle.
        delta_inner = np.arctan(self.L / (R - direction * self.W / 2))
        delta_outer = np.arctan(self.L / (R + direction * self.W / 2))
        return delta_inner, delta_outer

    def turning_radius(self, delta):
        # delta: steering angle (rad).
        # Returns: the turning radius (m) of the bicycle model.
        #
        # Formula: R = L / sin(|delta|).
        # This differs from the usual L/tan(delta) because this computes
        # the radius to the center of mass or the outer edge depending on
        # convention. Here it uses sin, giving a slightly larger radius
        # than the tan-based formula for the same delta.
        #
        # If delta is near 0, returns infinity (straight line).
        # Changing L scales the radius linearly:
        #   longer L = larger turning radius for the same steering angle.
        if abs(delta) < 0.001:
            return float("inf")
        # R = L/sin(|δ|): radius to the vehicle's outer edge (longer than
        # the tan-based formula which gives the rear-axle-centre radius).
        return self.L / np.sin(abs(delta))
