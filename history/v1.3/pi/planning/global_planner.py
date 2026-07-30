import math
from ..system.logger import log


class GlobalPlanner:
    def __init__(self):
        self.waypoints = []

    def plan_rectangle(self, width_m, height_m):
        self.waypoints = [
            (0, 0),
            (width_m, 0),
            (width_m, height_m),
            (0, height_m),
            (0, 0),
        ]
        log.info(f"Rectangle plan: {len(self.waypoints)} waypoints")
        return self.waypoints

    def plan_lap(self, track_width, track_length):
        return self.plan_rectangle(track_width, track_length)

    def get_target(self, index):
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None
