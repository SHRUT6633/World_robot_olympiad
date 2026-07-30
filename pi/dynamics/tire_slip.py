# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/dynamics/tire_slip.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Tire slip angle estimation and lateral force calculation
# =============================================================================

import numpy as np


class TireSlipEstimator:
    # Estimates tire slip angle and the resulting lateral tire force.
    # Tire slip occurs when the tire's direction of travel does not align
    # with its heading — the rubber deforms and generates a lateral force
    # that (at low slip angles) is proportional to the slip angle.
    #
    # This is important for dynamic models at higher speeds where the
    # no-slip kinematic assumption breaks down.

    def __init__(self, mu=0.8):
        # mu: tire-road friction coefficient.
        #   0.8 is typical for dry asphalt. Lower values (e.g., 0.3) represent
        #   wet/icy conditions where the tire saturates at smaller slip angles.
        #   Changing mu scales the maximum possible lateral force.
        self.mu = mu

    def estimate_slip_angle(self, vy, vx, yaw_rate, wheelbase_rear=0.13):
        # vy: lateral velocity in the body frame (m/s).
        # vx: longitudinal velocity in the body frame (m/s).
        # yaw_rate: angular velocity around the vertical axis (rad/s).
        # wheelbase_rear: distance from CG to the rear axle (m).
        #   This determines how much yaw contributes to the rear tire slip.
        # Returns: rear tire slip angle (rad).
        #
        # Slip angle formula for the rear tire:
        #   alpha_r = arctan2(vy - lr * yaw_rate, vx)
        # The numerator is the lateral velocity at the rear axle (including
        # the rotational component from yaw). The denominator is forward speed.
        #
        # If vx is near 0, return 0 (the tire is stationary — no slip defined).
        # Changing wheelbase_rear shifts the effective measurement point along
        # the vehicle, changing how yaw rate contributes to the slip estimate.
        if abs(vx) < 0.01:
            return 0.0
        return np.arctan2(vy - wheelbase_rear * yaw_rate, vx)

    def lateral_force(self, slip_angle):
        # slip_angle: tire slip angle (rad).
        # Returns: estimated lateral tire force (N).
        #
        # Uses a simple saturation model: F = -mu * tanh(10 * slip_angle).
        # The tanh function gives a linear region near zero (F ~ -10*mu*angle)
        # and saturates at ±mu for large slip angles.
        #
        # Changing mu scales the saturation level. The factor 10 controls how
        # quickly the force saturates — smaller = more gradual, larger = sharper
        # transition. At slip_angle ≈ 0.3 rad (~17°), tanh(3) ≈ 0.995, so the
        # tire is nearly fully saturated.
        return -self.mu * np.tanh(10 * slip_angle)
