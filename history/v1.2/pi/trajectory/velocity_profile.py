import numpy as np


class VelocityProfiler:
    def __init__(self, max_v=2.0, max_a=1.0):
        self.max_v = max_v
        self.max_a = max_a

    def trapezoidal(self, path, dt=0.01):
        n = len(path)
        v_profile = np.zeros(n)
        accel_steps = int(self.max_v / (self.max_a * dt))
        for i in range(n):
            if i < accel_steps:
                v_profile[i] = self.max_a * dt * i
            elif i > n - accel_steps:
                v_profile[i] = self.max_a * dt * (n - i - 1)
            else:
                v_profile[i] = self.max_v
        return np.clip(v_profile, 0, self.max_v)

    def curvature_limited(self, path, curvature, max_lat_a=2.0):
        v_curv = np.sqrt(max_lat_a / (curvature + 1e-6))
        return np.minimum(v_curv, self.max_v)
