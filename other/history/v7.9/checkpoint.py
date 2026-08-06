import math
import time
import logging


class Section:
    def __init__(self, section_id, name, waypoints, behavior="normal"):
        self.id = section_id
        self.name = name
        self.waypoints = waypoints
        self.behavior = behavior

    def __repr__(self):
        return f"Section({self.id}: {self.name})"


DEFAULT_SECTIONS = [
    Section(1, "Start/Finish Straight",
            [(0.0, 0.0), (1.5, 0.0)], "fast"),
    Section(2, "Outer Curve 1",
            [(1.5, 0.0), (2.5, 0.8)], "cornering"),
    Section(3, "Outer Straight",
            [(2.5, 0.8), (3.5, 0.8)], "fast"),
    Section(4, "Outer Curve 2",
            [(3.5, 0.8), (4.0, 1.8)], "cornering"),
    Section(5, "Technical Zone Entry",
            [(4.0, 1.8), (3.0, 2.2)], "slow"),
    Section(6, "Technical Zigzag",
            [(3.0, 2.2), (2.0, 1.8)], "slow"),
    Section(7, "Technical Zone Exit",
            [(2.0, 1.8), (1.0, 2.0)], "slow"),
    Section(8, "Return Straight",
            [(1.0, 2.0), (0.0, 0.0)], "fast"),
]


class Pose:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self):
        return f"Pose(x={self.x:.3f}, y={self.y:.3f})"


class CheckpointManager:
    def __init__(self, sections=None, look_ahead_distance=0.5,
                 transition_timeout=3.0, validation_distance=0.2):
        self.sections = sections or DEFAULT_SECTIONS
        self.look_ahead_distance = look_ahead_distance
        self.transition_timeout = transition_timeout
        self.validation_distance = validation_distance

        self._current_section = None
        self._last_transition_time = 0.0
        self._transition_pose = None
        self._transition_count = 0
        self._section_history = []
        self._lap_count = 0
        self._missed_sections = []

        self.logger = logging.getLogger(self.__class__.__name__)

    def update(self, pose):
        if self._current_section is None:
            self._find_initial_section(pose)
            return

        current = self._current_section
        remaining = self._distance_to_section_end(pose, current)

        if remaining < self.look_ahead_distance:
            next_section = self._find_next_section(current.id)
            if next_section:
                self._do_transition(next_section, pose)
            else:
                self._lap_completed(pose)

        elif self._transition_pose is not None:
            self._validate_transition(pose)

    def _find_initial_section(self, pose):
        best_section = None
        best_distance = float("inf")
        for section in self.sections:
            dist = self._distance_to_section(pose, section)
            if dist < best_distance:
                best_distance = dist
                best_section = section

        if best_section:
            self._current_section = best_section
            self._last_transition_time = time.monotonic()
            self._transition_pose = Pose(pose.x, pose.y, pose.yaw)
            self.logger.info(
                f"Initial section: {best_section.id} ({best_section.name})"
            )

    def _distance_to_section(self, pose, section):
        sx, sy = section.waypoints[0]
        dx = pose.x - sx
        dy = pose.y - sy
        return math.hypot(dx, dy)

    def _distance_to_section_end(self, pose, section):
        ex, ey = section.waypoints[-1]
        dx = ex - pose.x
        dy = ey - pose.y
        return math.hypot(dx, dy)

    def _find_next_section(self, current_id):
        next_id = current_id + 1
        for section in self.sections:
            if section.id == next_id:
                return section
        return None

    def _do_transition(self, next_section, pose):
        prev = self._current_section
        self._current_section = next_section
        self._transition_count += 1
        self._last_transition_time = time.monotonic()
        self._transition_pose = Pose(pose.x, pose.y, pose.yaw)
        self._section_history.append(next_section.id)
        self.logger.info(
            f"Section {prev.id} → {next_section.id} "
            f"({next_section.name})"
        )

    def _lap_completed(self, pose):
        self._lap_count += 1
        self._current_section = self.sections[0]
        self._section_history.clear()
        self.logger.info(
            f"Lap {self._lap_count} completed. "
            f"Restarting section tracking."
        )

    def _validate_transition(self, pose):
        elapsed = time.monotonic() - self._last_transition_time
        if elapsed > self.transition_timeout:
            dx = pose.x - self._transition_pose.x
            dy = pose.y - self._transition_pose.y
            distance = math.hypot(dx, dy)
            if distance < self.validation_distance:
                self.logger.warning(
                    f"Section {self._current_section.id} entered but "
                    f"only moved {distance:.2f}m in {elapsed:.1f}s"
                )

    def get_current_section(self):
        return self._current_section

    def get_current_section_id(self):
        if self._current_section is None:
            return None
        return self._current_section.id

    def get_current_behavior(self):
        if self._current_section is None:
            return "normal"
        return self._current_section.behavior

    def get_transition_count(self):
        return self._transition_count

    def get_lap_count(self):
        return self._lap_count

    def get_section_history(self):
        return list(self._section_history)

    def is_on_last_section(self):
        if self._current_section is None:
            return False
        return self._current_section.id == self.sections[-1].id

    def reset(self):
        self._current_section = None
        self._last_transition_time = 0.0
        self._transition_pose = None
        self._transition_count = 0
        self._section_history.clear()
        self._lap_count = 0
        self._missed_sections.clear()
