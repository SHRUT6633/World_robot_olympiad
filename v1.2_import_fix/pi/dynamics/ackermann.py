import numpy as np


class AckermannGeometry:
    def __init__(self, wheelbase=0.26, track_width=0.18):
        self.L = wheelbase
        self.W = track_width

    def inner_outer_angles(self, delta, direction=1):
        R = self.L / (np.tan(abs(delta)) + 1e-6)
        delta_inner = np.arctan(self.L / (R - direction * self.W / 2))
        delta_outer = np.arctan(self.L / (R + direction * self.W / 2))
        return delta_inner, delta_outer

    def turning_radius(self, delta):
        if abs(delta) < 0.001:
            return float("inf")
        return self.L / np.sin(abs(delta))
