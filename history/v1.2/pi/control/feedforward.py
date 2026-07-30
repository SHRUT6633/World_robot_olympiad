import numpy as np


class FeedforwardSteering:
    def __init__(self, wheelbase=0.26):
        self.L = wheelbase

    def compute(self, curvature, v=None):
        if abs(curvature) < 1e-6:
            return 0.0
        return np.arctan(self.L * curvature)
