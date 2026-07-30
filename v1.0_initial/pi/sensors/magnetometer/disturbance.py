import numpy as np
from ...system.logger import log


class MagneticDisturbanceDetector:
    def __init__(self, threshold=100, window=10):
        self.threshold = threshold
        self.window = window
        self._history = []
        self._baseline = None

    def detect(self, mag):
        magnitude = np.linalg.norm(mag)
        self._history.append(magnitude)
        if len(self._history) > self.window:
            self._history.pop(0)
        if len(self._history) < 5:
            return False
        mean = np.mean(self._history)
        std = np.std(self._history) + 1e-6
        disturbed = abs(magnitude - mean) > self.threshold * std
        return bool(disturbed)
