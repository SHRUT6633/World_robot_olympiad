import math
import logging
from enum import Enum


PASS_LEFT = "pass_left"
PASS_RIGHT = "pass_right"
PASS_DYNAMIC = "pass_dynamic"


class ManeuverState(Enum):
    ENTERING = "entering"
    AVOIDING = "avoiding"
    CLEARING = "clearing"
    COMPLETE = "complete"


class SensorData:
    def __init__(self, front=0.0, left=0.0, right=0.0,
                 front_left=0.0, front_right=0.0):
        self.front = front
        self.left = left
        self.right = right
        self.front_left = front_left
        self.front_right = front_right

    def __repr__(self):
        return (f"SensorData(f={self.front:.2f}, fl={self.front_left:.2f}, "
                f"fr={self.front_right:.2f}, l={self.left:.2f}, "
                f"r={self.right:.2f})")


class ObstacleManeuver:
    def __init__(self, strategy, entry_pose):
        self.strategy = strategy
        self.entry_pose = entry_pose
        self.state = ManeuverState.ENTERING
        self.locked = True
        self.start_time = 0.0
        self.cleared_time = 0.0

    def get_pass_side(self):
        return self.strategy


class ObstacleStrategy:
    def __init__(self, config=None):
        self.config = {
            "pass_side": PASS_DYNAMIC,
            "default_pass_side": PASS_LEFT,
            "min_pass_width": 0.25,
            "sensor_angle_deg": 45,
            "lock_on_entry": True,
            "abort_if_infeasible": True,
            "obstacle_detect_distance": 0.35,
        }
        if config:
            self.config.update(config)

        self.current_maneuver = None
        self.obstacle_detected = False
        self.last_decision_time = 0.0
        self.logger = logging.getLogger(self.__class__.__name__)

    def update(self, sensor_data, robot_pose):
        self.sensor_data = sensor_data
        self.robot_pose = robot_pose

        if self.current_maneuver is not None:
            self._update_maneuver(sensor_data)
            return self.current_maneuver

        detected = self._detect_obstacle(sensor_data)
        if detected:
            strategy = self._decide_strategy(sensor_data)
            self.current_maneuver = ObstacleManeuver(
                strategy=strategy,
                entry_pose=robot_pose,
            )
            self.logger.info(
                f"Obstacle detected. Strategy: {strategy}. "
                f"Left: {sensor_data.front_left:.2f}, "
                f"Right: {sensor_data.front_right:.2f}"
            )

        return self.current_maneuver

    def _detect_obstacle(self, sensor_data):
        return sensor_data.front < self.config["obstacle_detect_distance"]

    def _decide_strategy(self, sensor_data):
        mode = self.config["pass_side"]
        if mode == PASS_LEFT:
            return PASS_LEFT
        elif mode == PASS_RIGHT:
            return PASS_RIGHT
        elif mode == PASS_DYNAMIC:
            return self._decide_dynamic(sensor_data)
        else:
            return self.config["default_pass_side"]

    def _decide_dynamic(self, sensor_data):
        margin = 0.15
        angle_rad = math.radians(self.config["sensor_angle_deg"])
        left_clearance = sensor_data.front_left * math.sin(angle_rad)
        right_clearance = sensor_data.front_right * math.sin(angle_rad)
        left_space = left_clearance - margin
        right_space = right_clearance - margin
        if left_space > right_space and left_space > 0:
            return PASS_LEFT
        elif right_space > left_space and right_space > 0:
            return PASS_RIGHT
        else:
            return self.config["default_pass_side"]

    def _update_maneuver(self, sensor_data):
        if self.config["abort_if_infeasible"]:
            self._check_feasibility(sensor_data)
        if self.current_maneuver is None:
            return

        if self.current_maneuver.state == ManeuverState.ENTERING:
            if sensor_data.front_left > 0.5 and sensor_data.front_right > 0.5:
                self.current_maneuver.state = ManeuverState.CLEARING
            else:
                self.current_maneuver.state = ManeuverState.AVOIDING

        elif self.current_maneuver.state == ManeuverState.AVOIDING:
            if sensor_data.front > 0.5:
                self.current_maneuver.state = ManeuverState.CLEARING

        elif self.current_maneuver.state == ManeuverState.CLEARING:
            if sensor_data.front > 0.8:
                self.current_maneuver = None

    def _check_feasibility(self, sensor_data):
        if self.current_maneuver is None:
            return
        side = self.current_maneuver.strategy
        clearance = (
            sensor_data.left if side == PASS_LEFT else sensor_data.right
        )
        if clearance < self.config["min_pass_width"]:
            self.logger.warning(
                f"Maneuver {side} infeasible: clearance "
                f"{clearance:.2f}m < {self.config['min_pass_width']:.2f}m. "
                f"Aborting."
            )
            self.current_maneuver = None

    def is_maneuvering(self):
        return self.current_maneuver is not None

    def get_current_strategy(self):
        if self.current_maneuver is None:
            return None
        return self.current_maneuver.strategy

    def get_maneuver_state(self):
        if self.current_maneuver is None:
            return None
        return self.current_maneuver.state

    def reset(self):
        self.current_maneuver = None
        self.obstacle_detected = False
        self.last_decision_time = 0.0
