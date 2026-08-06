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
        # αᵣ = arctan2(vy - lr*ψ̇, vx): rear slip angle is the angle between
        # the tire's heading and its velocity vector at the rear axle.
        # The term -lr*ψ̇ is the lateral velocity contribution from yaw rotation.
        if abs(vx) < 0.01:
            return 0.0
        return np.arctan2(vy - wheelbase_rear * yaw_rate, vx)

    def lateral_force(self, slip_angle):
        # Fy = -μ*tanh(10*α): saturating lateral force model — linear at
        # small α (Fy ∝ -10*μ*α), saturating to ±μ at large α (≥0.3 rad).
        return -self.mu * np.tanh(10 * slip_angle)
