from collections import namedtuple

WallState = namedtuple("WallState", [
    "left_mm", "right_mm", "front_mm",
    "wall_dist_mm", "wall_angle_rad",
    "timestamp_s"
])

SENSOR_SPACING_MM = 160.0
TARGET_WALL_DIST_MM = 175.0
WALL_ANGLE_DEADBAND_RAD = 0.05
FRONT_OBSTACLE_THRESHOLD_MM = 500
