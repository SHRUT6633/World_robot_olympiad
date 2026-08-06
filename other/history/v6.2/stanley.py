import numpy as np


class StanleyController:
    def __init__(self, k=1.0, k_soft=1.0, max_steering=np.radians(30)):
        self.k = k
        self.k_soft = k_soft
        self.max_steering = max_steering

    def _select_k(self, v):
        if v >= 1.0:
            return 1.0
        elif v <= 0.3:
            return 0.5
        else:
            t = (v - 0.3) / 0.7
            return 0.5 + t * 0.5

    def nearest_point(self, x, y, path):
        dists = np.hypot(path[:, 0] - x, path[:, 1] - y)
        idx = int(np.argmin(dists))
        return path[idx], idx

    def compute(self, x, y, heading, path, v):
        target, idx = self.nearest_point(x, y, path)

        dx = target[0] - x
        dy = target[1] - y

        crosstrack = -np.sin(heading) * dx + np.cos(heading) * dy

        if idx + 1 < len(path):
            next_pt = path[min(idx + 1, len(path) - 1)]
            target_heading = np.arctan2(next_pt[1] - target[1], next_pt[0] - target[0])
        else:
            target_heading = heading

        heading_error = target_heading - heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        k = self._select_k(v)
        steer = heading_error + np.arctan2(k * crosstrack, self.k_soft + v)
        return float(np.clip(steer, -self.max_steering, self.max_steering))
