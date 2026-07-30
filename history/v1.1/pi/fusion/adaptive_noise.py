import numpy as np
from collections import deque


class AdaptiveNoiseEstimator:
    def __init__(self, window=100):
        self.window = window
        self._innovations = deque(maxlen=window)
        self._residuals = deque(maxlen=window)

    def update(self, innovation, residual):
        self._innovations.append(innovation)
        self._residuals.append(residual)

    def estimate_q(self, dim=6):
        if len(self._innovations) < 10:
            return np.eye(dim) * 0.001
        innov = np.array(self._innovations)
        cov = np.cov(innov, rowvar=False)
        Q = np.eye(dim) * np.mean(np.diag(cov)) * 0.5
        return Q

    def estimate_r(self, dim=6):
        if len(self._residuals) < 10:
            return np.eye(dim) * 0.1
        resid = np.array(self._residuals)
        cov = np.cov(resid, rowvar=False)
        R = np.eye(dim) * np.mean(np.diag(cov)) * 0.5
        return R
