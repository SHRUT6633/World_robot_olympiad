import numpy as np


class FeedforwardSteering:
    def __init__(self, wheelbase=0.26, max_ff_ratio=0.5, max_steering=np.radians(30)):
        self.L = wheelbase
        self.max_ff_ratio = max_ff_ratio
        self.max_steering = max_steering

        self._prev_curvature = 0.0

    def compute_feedforward(self, curvature):
        if abs(curvature) < 1e-6:
            return 0.0
        return np.arctan(self.L * curvature)

    def compute_curvature(self, path, idx):
        if idx < 1 or idx >= len(path) - 1:
            return 0.0

        p_prev = path[idx - 1]
        p_curr = path[idx]
        p_next = path[idx + 1]

        v1 = p_curr - p_prev
        v2 = p_next - p_curr
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        mag = np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8

        sin_theta = cross / mag
        r = 1.0 / (2.0 * abs(sin_theta) + 1e-8)
        curvature = np.sign(sin_theta) * (1.0 / r)
        curvature = 0.7 * curvature + 0.3 * self._prev_curvature
        self._prev_curvature = curvature
        return curvature

    def compute(self, feedback_steer, curvature):
        ff = self.compute_feedforward(curvature)
        total_raw = feedback_steer + ff
        max_ff_contrib = self.max_ff_ratio * abs(total_raw)
        ff_limited = np.clip(ff, -max_ff_contrib, max_ff_contrib)
        total = feedback_steer + ff_limited
        return float(np.clip(total, -self.max_steering, self.max_steering))
