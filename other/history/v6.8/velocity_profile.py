import numpy as np


class VelocityProfiler:
    def __init__(self, max_v=2.0, max_a=0.5):
        self.max_v = max_v
        self.max_a = max_a

    def curvature(self, path):
        n = len(path)
        kappa = np.zeros(n)
        for i in range(1, n - 1):
            v1 = path[i] - path[i - 1]
            v2 = path[i + 1] - path[i]
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            dot = v1[0] * v2[0] + v1[1] * v2[1]
            mag = np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8
            sin_theta = cross / mag
            r = 1.0 / (2.0 * abs(sin_theta) + 1e-8)
            kappa[i] = np.sign(sin_theta) * (1.0 / r)
        return kappa

    def curvature_limited(self, path, max_lat_a=2.0):
        kappa = self.curvature(path)
        v = np.sqrt(max_lat_a / (np.abs(kappa) + 1e-6))
        return np.minimum(v, self.max_v)

    def compute(self, path, dt=0.01):
        n = len(path)
        if n < 2:
            return np.array([self.max_v])

        v = self.curvature_limited(path)

        ds = 0.0
        for i in range(1, n):
            ds_i = np.linalg.norm(path[i] - path[i - 1])
            ds += ds_i
        ds /= n

        for i in range(1, n):
            v[i] = min(v[i], v[i - 1] + self.max_a * ds / (v[i - 1] + 1e-6))

        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], v[i + 1] + self.max_a * ds / (v[i + 1] + 1e-6))

        return np.clip(v, 0.01, self.max_v)

    def trapezoidal(self, path, dt=0.01):
        n = len(path)
        v = np.zeros(n)
        accel_steps = int(self.max_v / (self.max_a * dt))
        for i in range(n):
            if i < accel_steps:
                v[i] = self.max_a * dt * i
            elif i > n - accel_steps:
                v[i] = self.max_a * dt * (n - i - 1)
            else:
                v[i] = self.max_v
        return np.clip(v, 0, self.max_v)
