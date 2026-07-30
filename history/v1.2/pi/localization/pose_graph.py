import numpy as np
from ..system.logger import log


class PoseGraphOptimizer:
    def __init__(self):
        self.poses = []
        self.constraints = []

    def add_pose(self, x, y, heading):
        self.poses.append(np.array([x, y, heading]))

    def add_constraint(self, i, j, dx, dy, dheading, info=np.eye(3)):
        self.constraints.append((i, j, np.array([dx, dy, dheading]), info))

    def optimize(self, iterations=10):
        if len(self.poses) < 2:
            return self.poses
        poses = [p.copy() for p in self.poses]
        for _ in range(iterations):
            for i, j, rel, info in self.constraints:
                if i >= len(poses) or j >= len(poses):
                    continue
                error = poses[j] - poses[i] - rel
                error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
                correction = 0.1 * info @ error
                poses[j] -= correction
        self.poses = poses
        return self.poses
