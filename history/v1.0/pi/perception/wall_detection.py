import numpy as np


class WallDetector:
    def __init__(self, threshold_mm=500):
        self.threshold = threshold_mm

    def detect(self, left_dist, right_dist, front_dist):
        walls = {"left": False, "right": False, "front": False}
        if left_dist is not None and left_dist < self.threshold:
            walls["left"] = True
        if right_dist is not None and right_dist < self.threshold:
            walls["right"] = True
        if front_dist is not None and front_dist < self.threshold:
            walls["front"] = True
        return walls
