import math
import time


SENSOR_SEPARATION_MM = 150.0
MIN_VALID_DISTANCE = 35.0
MAX_CONSECUTIVE_ZERO = 10


class WallDetect:
    def __init__(self, tof_left, tof_right):
        self.tof_left = tof_left
        self.tof_right = tof_right
        self.consecutive_zero = 0

    def read_tof(self, sensor):
        dist = sensor.get_distance()
        status = sensor.get_range_status()
        if status != 0 or dist < MIN_VALID_DISTANCE:
            return 0.0
        return float(dist)

    def measure(self):
        left_dist = self.read_tof(self.tof_left)
        time.sleep(0.01)
        right_dist = self.read_tof(self.tof_right)

        if left_dist == 0.0 and right_dist == 0.0:
            self.consecutive_zero += 1
        else:
            self.consecutive_zero = 0

        if self.consecutive_zero > MAX_CONSECUTIVE_ZERO:
            return {"distance": 0.0, "angle": 0.0, "wedged": True}

        wall_angle = math.atan2(
            right_dist - left_dist, SENSOR_SEPARATION_MM
        )

        wall_distance = (left_dist + right_dist) / 2.0

        return {
            "distance": wall_distance,
            "angle": wall_angle,
            "left_dist": left_dist,
            "right_dist": right_dist,
            "wedged": False,
        }
