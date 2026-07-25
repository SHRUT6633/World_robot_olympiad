import math
from ..system.logger import log


class CheckpointManager:
    def __init__(self):
        self.checkpoints = []
        self._current = 0

    def add(self, x, y, threshold=0.2):
        self.checkpoints.append({"x": x, "y": y, "threshold": threshold, "reached": False})

    def check_reached(self, robot_x, robot_y):
        if self._current >= len(self.checkpoints):
            return False
        cp = self.checkpoints[self._current]
        dist = math.sqrt((robot_x - cp["x"])**2 + (robot_y - cp["y"])**2)
        if dist < cp["threshold"] and not cp["reached"]:
            cp["reached"] = True
            self._current += 1
            log.info(f"Checkpoint {self._current}/{len(self.checkpoints)} reached")
            return True
        return False

    def next_target(self):
        if self._current < len(self.checkpoints):
            cp = self.checkpoints[self._current]
            return cp["x"], cp["y"]
        return None

    @property
    def all_reached(self):
        return self._current >= len(self.checkpoints)

    def reset(self):
        self._current = 0
        for cp in self.checkpoints:
            cp["reached"] = False
