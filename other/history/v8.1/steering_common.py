import math
from dataclasses import dataclass


@dataclass
class SteeringCommand:
    front_left: float
    front_right: float
    rear_left: float
    rear_right: float
    turning_radius: float
    mode: str


@dataclass
class SteeringLimits:
    max_angle: float
    min_turning_radius: float


def validate_steering_angle(angle: float, limits: SteeringLimits) -> float:
    if abs(angle) > limits.max_angle:
        angle = math.copysign(limits.max_angle, angle)
    return angle


def rad_to_servo_pulse(angle_rad: float) -> int:
    angle_deg = math.degrees(angle_rad)
    pulse = int(1500 + (angle_deg / 90.0) * 500)
    return max(1000, min(2000, pulse))


def compute_speed_for_mode(mode: str, requested_speed: float, limits: dict[str, float]) -> float:
    cap = limits.get(mode, requested_speed)
    return min(requested_speed, cap)
