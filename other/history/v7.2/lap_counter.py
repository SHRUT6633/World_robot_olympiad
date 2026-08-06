import math
import time
import logging


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Pose:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self):
        return f"Pose(x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°)"


class LapCounter:
    def __init__(self, line_position, zone_radius=0.30, clear_distance=0.50,
                 expected_heading=0.0, max_laps=3):
        self.line_position = line_position
        self.zone_radius = zone_radius
        self.clear_distance = clear_distance
        self.expected_heading = expected_heading
        self.max_laps = max_laps

        self.lap_count = 0
        self.armed = True
        self.last_lap_time = 0.0
        self.lap_times = []
        self._was_in_zone = False

        self.logger = logging.getLogger(self.__class__.__name__)

    def process_pose(self, pose):
        dx = pose.x - self.line_position.x
        dy = pose.y - self.line_position.y
        distance = math.hypot(dx, dy)
        in_zone = distance < self.zone_radius

        if in_zone and self._was_in_zone:
            return False

        self._was_in_zone = in_zone

        if distance > self.clear_distance:
            self.armed = True
            return False

        if in_zone and self.armed:
            heading_error = abs(normalize_angle(
                pose.yaw - self.expected_heading
            ))
            if heading_error > math.radians(45):
                self.logger.warning(
                    f"Line crossed at invalid heading: "
                    f"{math.degrees(pose.yaw):.1f}° "
                    f"(error {math.degrees(heading_error):.1f}°)"
                )
                return False

            self.lap_count += 1
            now = time.time()
            if self.last_lap_time > 0:
                self.lap_times.append(now - self.last_lap_time)
            self.last_lap_time = now
            self.armed = False

            self.logger.info(
                f"Lap {self.lap_count}/{self.max_laps} completed "
                f"at t={now:.2f}s"
            )
            return True

        return False

    def is_finished(self):
        return self.lap_count >= self.max_laps

    def get_lap_count(self):
        return self.lap_count

    def get_remaining_laps(self):
        return max(0, self.max_laps - self.lap_count)

    def get_last_lap_time(self):
        if not self.lap_times:
            return 0.0
        return self.lap_times[-1]

    def get_average_lap_time(self):
        if not self.lap_times:
            return 0.0
        return sum(self.lap_times) / len(self.lap_times)

    def reset(self):
        self.lap_count = 0
        self.armed = True
        self.last_lap_time = 0.0
        self.lap_times.clear()
        self._was_in_zone = False
