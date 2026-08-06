import math
from dataclasses import dataclass

from history.v8_common.steering_common import (
    SteeringCommand,
    SteeringLimits,
    validate_steering_angle,
)


@dataclass
class SamePhaseSteeringConfig:
    max_steering_angle_deg: float = 25.0
    wheelbase_m: float = 0.45
    track_width_m: float = 0.36
    min_turning_radius_m: float = 0.8


class SamePhaseSteering:
    def __init__(self, config: SamePhaseSteeringConfig | None = None):
        self.config = config or SamePhaseSteeringConfig()
        self.limits = SteeringLimits(
            max_angle=math.radians(self.config.max_steering_angle_deg),
            min_turning_radius=self.config.min_turning_radius_m,
        )

    def compute_steering(self, turning_radius: float) -> SteeringCommand:
        if turning_radius < self.config.min_turning_radius_m:
            turning_radius = self.config.min_turning_radius_m

        angle = math.atan2(self.config.wheelbase_m, 2.0 * turning_radius)
        angle = validate_steering_angle(angle, self.limits)

        return SteeringCommand(
            front_left=angle,
            front_right=angle,
            rear_left=angle,
            rear_right=angle,
            turning_radius=turning_radius,
            mode="same_phase",
        )

    def stop(self) -> SteeringCommand:
        return SteeringCommand(
            front_left=0.0,
            front_right=0.0,
            rear_left=0.0,
            rear_right=0.0,
            turning_radius=float("inf"),
            mode="same_phase",
        )
