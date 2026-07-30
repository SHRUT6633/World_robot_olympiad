import math
from dataclasses import dataclass


PARKING_ZONE_WIDTH_M = 0.5
PARKING_ZONE_LENGTH_M = 0.3


@dataclass
class Pose2D:
    x: float
    y: float
    theta_deg: float


def compute_alignment_error(robot_pose: Pose2D, zone: "ParkingZone") -> float:
    return abs(robot_pose.theta_deg - zone.orientation_deg)


def compute_distance_error(robot_pose: Pose2D, zone: "ParkingZone") -> float:
    dx = robot_pose.x - zone.center_x_m
    dy = robot_pose.y - zone.center_y_m
    return math.hypot(dx, dy)


def is_within_parallel_threshold(
    left_dist_m: float, right_dist_m: float, threshold_mm: float = 20.0
) -> bool:
    diff_mm = abs(left_dist_m - right_dist_m) * 1000.0
    return diff_mm <= threshold_mm


def compute_entry_vector(zone: "ParkingZone") -> tuple[float, float]:
    rad = math.radians(zone.orientation_deg)
    return (math.cos(rad), math.sin(rad))
