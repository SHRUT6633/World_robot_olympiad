# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/fusion/mahalanobis.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Mahalanobis distance outlier rejection
# =============================================================================

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
    #
    # Why needed: On the WRO track, sensors can produce gross outliers — GPS
    # multipath near walls, ultrasonic crosstalk, magnetometer hard-iron spikes.
    # Without rejection, a single bad measurement can cause the Kalman filter
    # to diverge catastrophically. The Mahalanobis gate provides a principled,
    # covariance-aware outlier test that respects each sensor's uncertainty.
    #
    # For a d-dimensional measurement, the chi-squared threshold should be
    # scaled: at 3 sigma, chi2.ppf(0.997, d) gives the correct gate.
    # Here we keep a single scalar threshold for simplicity (works well for
    # our mostly-1D sensor channels).

    def __init__(self, threshold=3.0):
        self.threshold = threshold   # Mahalanobis distance threshold for rejection
        self._history = []           # (unused in current implementation, reserved)

    def test(self, z, mean, cov):
        # Compute squared Mahalanobis distance for a multivariate measurement.
        # Returns True if the distance <= threshold (i.e., measurement is accepted as inlier).
        # d² = (z - μ)^T * Σ⁻¹ * (z - μ)
        #   where Σ = innovation covariance S in the filter context.
        #   d² ~ χ²(dim) under the null hypothesis (valid measurement).
        delta = z - mean
        try:
            # d² = delta^T * cov^{-1} * delta
            # O(d^3) via direct inverse — fine for d <= 6 where d = dim of sub-block
            d_sq = delta @ np.linalg.inv(cov) @ delta
        except np.linalg.LinAlgError:
            # If covariance is singular (e.g., filter not yet initialised),
            # accept the measurement rather than crash
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
        # Rejected dimensions are replaced with the predicted value in the filter,
        # effectively ignoring the bad sensor channel while still using good ones.
        mask = np.ones(len(z), dtype=bool)
        for i in range(len(z)):
            zi = z[i]
            pred_i = predicted_measurement[i]
            if np.isscalar(zi):
                # 1D measurement: use normalized absolute deviation
                # Equivalent to Mahalanobis distance for a scalar:
                #   d = |z_i - pred_i| / sqrt(S[i,i])
                # Add epsilon to prevent division by zero when S[i,i] ~ 0
                # (e.g., filter has fully converged on that channel).
                d = abs(zi - pred_i) / (np.sqrt(S[i, i]) + 1e-6)
                if d > self.threshold:
                    mask[i] = False
            else:
                # Multi-dimensional sub-measurement (e.g., 3D position):
                # test using full sub-block of S
                # Assumes sub-blocks are contiguous in the measurement vector
                # and each sub-block is exactly 3 elements (config-dependent).
                if not self.test(zi, pred_i, S[i*3:(i+1)*3, i*3:(i+1)*3]):
                    mask[i] = False
        return mask
