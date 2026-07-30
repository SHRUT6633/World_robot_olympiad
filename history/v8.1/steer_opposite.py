import math
from dataclasses import dataclass

from history.v8_common.steering_common import (
    SteeringCommand,
    SteeringLimits,
    validate_steering_angle,
)


@dataclass
class OppositePhaseSteeringConfig:
    max_steering_angle_deg: float = 30.0
    wheelbase_m: float = 0.45
    min_turning_radius_m: float = 0.5
    max_speed_mps: float = 0.3


class OppositePhaseSteering:
    def __init__(self, config: OppositePhaseSteeringConfig | None = None):
        self.config = config or OppositePhaseSteeringConfig()
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
            rear_left=-angle,
            rear_right=-angle,
            turning_radius=turning_radius,
            mode="opposite_phase",
        )

    def get_speed_limit(self) -> float:
        return self.config.max_speed_mps

    def stop(self) -> SteeringCommand:
        return SteeringCommand(
            front_left=0.0,
            front_right=0.0,
            rear_left=0.0,
            rear_right=0.0,
            turning_radius=float("inf"),
            mode="opposite_phase",
        )
