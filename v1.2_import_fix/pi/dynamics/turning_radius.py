import numpy as np


class TurningRadiusPredictor:
    def __init__(self, wheelbase=0.26):
        self.L = wheelbase

    def predict(self, delta):
        if abs(delta) < 1e-6:
            return float("inf")
        return self.L / abs(np.tan(delta))

    def min_radius(self, max_steering):
        return self.L / abs(np.tan(max_steering))
