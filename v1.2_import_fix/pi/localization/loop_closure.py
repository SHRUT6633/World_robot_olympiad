import numpy as np
from ..system.logger import log


class LoopClosureDetector:
    def __init__(self, distance_threshold=0.3, angle_threshold=15):
        self.dist_thresh = distance_threshold
        self.angle_thresh = np.radians(angle_threshold)

    def detect(self, current_pose, trajectory):
        cx, cy, cheading = current_pose
        for i, (x, y, heading) in enumerate(trajectory):
            d = np.sqrt((cx - x)**2 + (cy - y)**2)
            a = abs(cheading - heading)
            if d < self.dist_thresh and a < self.angle_thresh:
                return i
        return None
