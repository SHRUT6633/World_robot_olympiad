import numpy as np


class KinematicModel:
    def __init__(self, wheelbase=0.26):
        self.L = wheelbase

    def update(self, x, y, heading, v, delta, dt):
        x_next = x + v * np.cos(heading) * dt
        y_next = y + v * np.sin(heading) * dt
        heading_next = heading + (v / self.L) * np.tan(delta) * dt
        return x_next, y_next, heading_next

    def compute_steering(self, v, yaw_rate):
        if abs(v) < 0.01:
            return 0.0
        return np.arctan(self.L * yaw_rate / v)
