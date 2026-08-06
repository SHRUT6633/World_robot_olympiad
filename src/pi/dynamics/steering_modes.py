# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/dynamics/steering_modes.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: 4WS steering mode definitions and angle computation
# =============================================================================

import numpy as np
from enum import Enum


class SteeringMode(Enum):
    SAME_PHASE = "SAME_PHASE"
    OPPOSITE_PHASE = "OPPOSITE_PHASE"
    CRAB_WALK = "CRAB_WALK"


def compute_4ws_angles(steering_input_rad, mode: SteeringMode, max_steering_rad=np.radians(30)):
    # SAME_PHASE: all four wheels steer in the same direction — reduces effective
    # steering angle difference, producing a larger turning radius for stability.
    if mode == SteeringMode.SAME_PHASE:
        front = np.clip(steering_input_rad, -max_steering_rad, max_steering_rad)
        rear = front
    # OPPOSITE_PHASE: rear wheels counter-steer — doubles the effective angle
    # difference (front - rear = 2*delta), enabling tighter turns at low speed.
    elif mode == SteeringMode.OPPOSITE_PHASE:
        clipped = np.clip(steering_input_rad, -max_steering_rad, max_steering_rad)
        front = clipped
        rear = -clipped
    # CRAB_WALK: front and rear steer identically — zero yaw contribution
    # (front - rear = 0), making the robot translate diagonally.
    elif mode == SteeringMode.CRAB_WALK:
        clipped = np.clip(steering_input_rad, -max_steering_rad, max_steering_rad)
        front = clipped
        rear = clipped
    else:
        front = 0.0
        rear = 0.0
    turning_radius = _turning_radius(front, rear, wheelbase=0.26)
    return front, rear, turning_radius


def _turning_radius(front_angle_rad, rear_angle_rad, wheelbase=0.26):
    # Effective angle difference determines the turn curvature.
    diff = front_angle_rad - rear_angle_rad
    if abs(diff) < 1e-6:
        return float("inf")
    # R = L / |tan(δf) - tan(δr)| extends the bicycle model to 4WS:
    # opposite-phase steering produces a smaller radius than same-phase.
    return wheelbase / abs(np.tan(front_angle_rad) - np.tan(rear_angle_rad))
