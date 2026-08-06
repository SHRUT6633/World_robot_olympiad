import math
import time
import logging
from enum import Enum


CW = "clockwise"
CCW = "counter_clockwise"


class DirectionState(Enum):
    UNKNOWN = "unknown"
    DETECTING = "detecting"
    DETECTED = "detected"


class Pose:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self):
        return (f"Pose(x={self.x:.3f}, y={self.y:.3f}, "
                f"yaw={math.degrees(self.yaw):.1f}°)")


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class DirectionDetector:
    def __init__(self, min_detect_distance=0.5, yaw_threshold_deg=45,
                 yaw_buffer_size=5):
        self.min_detect_distance = min_detect_distance
        self.yaw_threshold = math.radians(yaw_threshold_deg)
        self.yaw_buffer_size = yaw_buffer_size

        self.state = DirectionState.UNKNOWN
        self.direction = None
        self._start_pose = None
        self._start_yaw = None
        self._yaw_buffer = []
        self._detection_time = 0.0
        self._max_yaw_change = 0.0

        self.logger = logging.getLogger(self.__class__.__name__)

    def process_pose(self, pose):
        if self.state == DirectionState.DETECTED:
            return self.direction

        if self._start_pose is None:
            self._start_pose = Pose(pose.x, pose.y, pose.yaw)
            self._start_yaw = pose.yaw
            self.state = DirectionState.DETECTING
            return None

        dx = pose.x - self._start_pose.x
        dy = pose.y - self._start_pose.y
        distance = math.hypot(dx, dy)

        if distance < self.min_detect_distance:
            return None

        self._yaw_buffer.append(pose.yaw)
        if len(self._yaw_buffer) > self.yaw_buffer_size:
            self._yaw_buffer.pop(0)

        avg_yaw = sum(self._yaw_buffer) / len(self._yaw_buffer)
        yaw_change = normalize_angle(avg_yaw - self._start_yaw)
        abs_change = abs(yaw_change)
        self._max_yaw_change = max(self._max_yaw_change, abs_change)

        if abs_change > self.yaw_threshold:
            self.direction = CW if yaw_change < 0 else CCW
            self.state = DirectionState.DETECTED
            self._detection_time = time.time()
            self.logger.info(
                f"Direction detected: {self.direction} "
                f"(yaw change: {math.degrees(yaw_change):.1f}°)"
            )
            return self.direction

        return None

    def get_direction(self):
        return self.direction

    def get_state(self):
        return self.state

    def is_detected(self):
        return self.state == DirectionState.DETECTED

    def get_detection_time(self):
        return self._detection_time

    def get_max_yaw_change(self):
        return self._max_yaw_change

    def reset(self):
        self.state = DirectionState.UNKNOWN
        self.direction = None
        self._start_pose = None
        self._start_yaw = None
        self._yaw_buffer.clear()
        self._detection_time = 0.0
        self._max_yaw_change = 0.0

    def force_direction(self, direction):
        if direction not in (CW, CCW):
            raise ValueError(f"Invalid direction: {direction}")
        self.direction = direction
        self.state = DirectionState.DETECTED
        self._detection_time = time.time()
        self.logger.info(f"Direction forced: {direction}")
