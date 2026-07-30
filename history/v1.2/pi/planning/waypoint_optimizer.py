import numpy as np


class WaypointOptimizer:
    def __init__(self, curvature_weight=0.1):
        self.curvature_weight = curvature_weight

    def optimize(self, waypoints):
        if len(waypoints) < 4:
            return waypoints
        pts = np.array(waypoints)
        opt = pts.copy()
        for _ in range(20):
            for i in range(1, len(pts) - 1):
                smooth = 0.5 * (opt[i - 1] + opt[i + 1])
                orig = pts[i]
                opt[i] = (1 - self.curvature_weight) * orig + self.curvature_weight * smooth
        return [(p[0], p[1]) for p in opt]
