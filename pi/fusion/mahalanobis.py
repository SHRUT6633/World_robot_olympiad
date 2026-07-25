import numpy as np
from ..system.logger import log


class MahalanobisOutlierRejector:
    def __init__(self, threshold=3.0):
        self.threshold = threshold
        self._history = []

    def test(self, z, mean, cov):
        delta = z - mean
        try:
            d_sq = delta @ np.linalg.inv(cov) @ delta
        except np.linalg.LinAlgError:
            return True
        return d_sq <= self.threshold ** 2

    def reject_outliers(self, z, predicted_measurement, S):
        mask = np.ones(len(z), dtype=bool)
        for i in range(len(z)):
            zi = z[i]
            pred_i = predicted_measurement[i]
            if np.isscalar(zi):
                d = abs(zi - pred_i) / (np.sqrt(S[i, i]) + 1e-6)
                if d > self.threshold:
                    mask[i] = False
            else:
                if not self.test(zi, pred_i, S[i*3:(i+1)*3, i*3:(i+1)*3]):
                    mask[i] = False
        return mask
