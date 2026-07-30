import time
from enum import Enum, auto
from ..system.logger import log


class RobotState(Enum):
    INIT = auto()
    IDLE = auto()
    FORWARD = auto()
    CORNERING = auto()
    OBSTACLE_AVOID = auto()
    REVERSE = auto()
    PARK = auto()
    FINISHED = auto()
    SHUTDOWN = auto()


class StateMachine:
    def __init__(self):
        self.state = RobotState.INIT
        self._avoid_start_time = 0
        self._park_entry_time = 0
        self._park_ready = False

    def transition(self, event):
        if event == "start_signal" and self.state == RobotState.INIT:
            self.state = RobotState.IDLE
            log.info("State: INIT -> IDLE")
        elif event == "track_detected" and self.state == RobotState.IDLE:
            self.state = RobotState.FORWARD
            log.info("State: IDLE -> FORWARD")
        elif event == "pillar_detected" and self.state == RobotState.FORWARD:
            self.state = RobotState.OBSTACLE_AVOID
            self._avoid_start_time = time.monotonic()
            log.info("State: FORWARD -> OBSTACLE_AVOID")
        elif event == "avoid_complete" and self.state == RobotState.OBSTACLE_AVOID:
            self.state = RobotState.FORWARD
            log.info("State: OBSTACLE_AVOID -> FORWARD")
        elif event == "lap_complete" and self.state == RobotState.FORWARD:
            self.state = RobotState.PARK
            self._park_entry_time = time.monotonic()
            self._park_ready = False
            log.info("State: FORWARD -> PARK")
        elif event == "park_success" and self.state == RobotState.PARK:
            self.state = RobotState.FINISHED
            log.info("State: PARK -> FINISHED")
        elif event == "shutdown":
            self.state = RobotState.SHUTDOWN
            log.info("State: SHUTDOWN")

    def update_parking(self, velocity, aligned):
        if self.state != RobotState.PARK:
            return
        if not self._park_ready:
            if abs(velocity) < 0.01:
                self._park_ready = True
                self._park_entry_time = time.monotonic()
        else:
            elapsed = time.monotonic() - self._park_entry_time
            if elapsed >= 30.0 and aligned:
                self.transition("park_success")

    def check_avoid_timeout(self):
        if self.state == RobotState.OBSTACLE_AVOID:
            elapsed = time.monotonic() - self._avoid_start_time
            if elapsed > 10.0:
                log.warn("Avoid timeout -> REVERSE")
                self.state = RobotState.REVERSE
                return True
        return False
