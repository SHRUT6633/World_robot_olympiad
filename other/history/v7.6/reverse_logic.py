import math
import time
import logging
from enum import Enum


class ReverseState(Enum):
    IDLE = "idle"
    REVERSING = "reversing"
    STOPPING = "stopping"
    RECOVERING = "recovering"


class ReverseResult(Enum):
    CONTINUE_REVERSE = "continue_reverse"
    STOP_REVERSE = "stop_reverse"
    ABORT_REVERSE = "abort_reverse"


class Pose:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self):
        return (f"Pose(x={self.x:.3f}, y={self.y:.3f}, "
                f"yaw={math.degrees(self.yaw):.1f}°)")


class ReverseLogic:
    def __init__(self, stuck_timeout=2.0, progress_threshold=0.02,
                 max_reverse_distance=0.20, rear_safety_margin=0.15,
                 recovery_turn_deg=20):
        self.stuck_timeout = stuck_timeout
        self.progress_threshold = progress_threshold
        self.max_reverse_distance = max_reverse_distance
        self.rear_safety_margin = rear_safety_margin
        self.recovery_turn_deg = recovery_turn_deg

        self.reverse_state = ReverseState.IDLE
        self._last_progress_pose = None
        self._last_progress_time = 0.0
        self._reverse_start_pose = None
        self._reverse_start_time = 0.0
        self._reverse_distance = 0.0
        self._stuck_count = 0

        self.logger = logging.getLogger(self.__class__.__name__)

    def check_stuck(self, pose):
        if self.reverse_state != ReverseState.IDLE:
            return False

        if self._last_progress_pose is None:
            self._last_progress_pose = Pose(pose.x, pose.y, pose.yaw)
            self._last_progress_time = time.monotonic()
            return False

        dx = pose.x - self._last_progress_pose.x
        dy = pose.y - self._last_progress_pose.y
        distance = math.hypot(dx, dy)
        elapsed = time.monotonic() - self._last_progress_time

        if distance > self.progress_threshold:
            self._last_progress_pose = Pose(pose.x, pose.y, pose.yaw)
            self._last_progress_time = time.monotonic()
            return False

        if elapsed > self.stuck_timeout:
            self._stuck_count += 1
            self.logger.warning(
                f"Stuck detected! No progress for {elapsed:.1f}s "
                f"(moved {distance:.3f}m). Count: {self._stuck_count}"
            )
            return True

        return False

    def start_reverse(self, pose):
        self.reverse_state = ReverseState.REVERSING
        self._reverse_start_pose = Pose(pose.x, pose.y, pose.yaw)
        self._reverse_start_time = time.monotonic()
        self._reverse_distance = 0.0
        self.logger.info(
            f"Starting reverse. ",
            f"Max distance: {self.max_reverse_distance:.2f}m, ",
            f"Rear margin: {self.rear_safety_margin:.2f}m",
        )

    def execute_reverse(self, pose, rear_distance=None):
        if self.reverse_state != ReverseState.REVERSING:
            return ReverseResult.STOP_REVERSE

        dx = pose.x - self._reverse_start_pose.x
        dy = pose.y - self._reverse_start_pose.y
        self._reverse_distance = math.hypot(dx, dy)

        if self._reverse_distance >= self.max_reverse_distance:
            self.logger.info(
                f"Max reverse distance reached "
                f"({self._reverse_distance:.2f}m)"
            )
            self.reverse_state = ReverseState.STOPPING
            return ReverseResult.STOP_REVERSE

        if rear_distance is not None:
            if rear_distance < self.rear_safety_margin:
                self.logger.warning(
                    f"Rear obstacle at {rear_distance:.2f}m. "
                    f"Aborting reverse."
                )
                self.reverse_state = ReverseState.STOPPING
                return ReverseResult.ABORT_REVERSE

        return ReverseResult.CONTINUE_REVERSE

    def get_recovery_turn(self):
        if self._stuck_count % 2 == 0:
            return math.radians(self.recovery_turn_deg)
        else:
            return math.radians(-self.recovery_turn_deg)

    def on_reverse_complete(self):
        self.reverse_state = ReverseState.RECOVERING
        self._last_progress_pose = None
        self._last_progress_time = 0.0

    def on_recovery_complete(self):
        self.reverse_state = ReverseState.IDLE

    def is_reversing(self):
        return self.reverse_state == ReverseState.REVERSING

    def is_recovering(self):
        return self.reverse_state == ReverseState.RECOVERING

    def is_active(self):
        return self.reverse_state != ReverseState.IDLE

    def get_stuck_count(self):
        return self._stuck_count

    def get_reverse_distance(self):
        return self._reverse_distance

    def reset(self):
        self.reverse_state = ReverseState.IDLE
        self._last_progress_pose = None
        self._last_progress_time = 0.0
        self._reverse_start_pose = None
        self._reverse_start_time = 0.0
        self._reverse_distance = 0.0
        self._stuck_count = 0
