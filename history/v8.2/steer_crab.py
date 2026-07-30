import math
from dataclasses import dataclass

from history.v8_common.steering_common import (
    SteeringCommand,
    SteeringLimits,
    validate_steering_angle,
)


@dataclass
class CrabWalkSteeringConfig:
    max_steering_angle_deg: float = 45.0
    max_speed_mps: float = 0.5


class CrabWalkSteering:
    def __init__(self, config: CrabWalkSteeringConfig | None = None):
        self.config = config or CrabWalkSteeringConfig()
        self.limits = SteeringLimits(
            max_angle=math.radians(self.config.max_steering_angle_deg),
            min_turning_radius=0.0,
        )

    def compute_steering(self, crab_angle_deg: float) -> SteeringCommand:
        angle = validate_steering_angle(
            math.radians(crab_angle_deg), self.limits
        )

        return SteeringCommand(
            front_left=angle,
            front_right=angle,
            rear_left=angle,
            rear_right=angle,
            turning_radius=float("inf"),
            mode="crab_walk",
        )

    def get_speed_limit(self) -> float:
        return self.config.max_speed_mps

    def decompose_velocity(self, speed: float, crab_angle_deg: float):
        rad = math.radians(crab_angle_deg)
        return speed * math.cos(rad), speed * math.sin(rad)

    def stop(self) -> SteeringCommand:
        return SteeringCommand(
            front_left=0.0,
            front_right=0.0,
            rear_left=0.0,
            rear_right=0.0,
            turning_radius=float("inf"),
            mode="crab_walk",
        )
