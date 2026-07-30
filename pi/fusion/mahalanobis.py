import numpy as np
from ..system.logger import log


class MahalanobisOutlierRejector:
    # Detects and rejects outlier measurements using the Mahalanobis distance.
    #
    # Mahalanobis distance measures how many standard deviations a measurement is
    # from the expected value, accounting for the covariance structure:
    #   d² = (z - mean)^T * cov⁻¹ * (z - mean)
    #
    # Under Gaussian assumptions, d² follows a chi-squared distribution.
    # threshold=3.0 corresponds to ~3 sigma (99.7% inlier rate for 1D).

    def __init__(self, threshold=3.0):
        self.threshold = threshold   # Mahalanobis distance threshold for rejection
        self._history = []           # (unused in current implementation, reserved)

    def test(self, z, mean, cov):
        # Compute squared Mahalanobis distance for a multivariate measurement.
        # Returns True if the distance <= threshold (i.e., measurement is accepted as inlier).
        delta = z - mean
        try:
            d_sq = delta @ np.linalg.inv(cov) @ delta
        except np.linalg.LinAlgError:
            # If covariance is singular, accept the measurement rather than crash
            return True
        return d_sq <= self.threshold ** 2

    def reject_outliers(self, z, predicted_measurement, S):
        # Vectorized outlier rejection: checks each element (or sub-vector) of z.
        #
        # z: full measurement vector (length depends on sensor config)
        # predicted_measurement: H*x_pred, same shape as z
        # S: innovation covariance matrix (H*P_pred*H^T + R)
        #
        # Returns a boolean mask of same length as z: True = keep, False = reject.
        mask = np.ones(len(z), dtype=bool)
        for i in range(len(z)):
            zi = z[i]
            pred_i = predicted_measurement[i]
            if np.isscalar(zi):
                # 1D measurement: use normalized absolute deviation
                d = abs(zi - pred_i) / (np.sqrt(S[i, i]) + 1e-6)
                if d > self.threshold:
                    mask[i] = False
            else:
                # Multi-dimensional sub-measurement (e.g., 3D position):
                # test using full sub-block of S
                if not self.test(zi, pred_i, S[i*3:(i+1)*3, i*3:(i+1)*3]):
                    mask[i] = False
        return mask
