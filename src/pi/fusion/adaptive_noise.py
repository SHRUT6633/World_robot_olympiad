# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/fusion/adaptive_noise.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Adaptive noise estimation for Kalman filters
# =============================================================================

import numpy as np
from collections import deque


class AdaptiveNoiseEstimator:
    # Estimates process noise (Q) and measurement noise (R) from recent filter statistics.
    #
    # The idea: the innovation (z - H*x_pred) and residual (z - H*x_upd) sequences carry
    # information about the actual noise present. If innovations are large/variable, Q or R
    # may be too small, causing the filter to under-trust measurements or over-trust predictions.
    #
    # By computing the sample covariance of a sliding window of innovations/residuals,
    # we can adjust Q and R at runtime to match the real noise, improving filter
    # accuracy when conditions change (e.g., rougher terrain, different speeds).
    #
    # Theoretical basis (Mehra 1970, "On the Identification of Variances..."):
    #   For an optimal Kalman filter, the innovation sequence should be white.
    #   If the sample innovation covariance deviates from S = H*P*H^T + R,
    #   either Q or R is mismatched. We adapt R from residuals (updates) and Q
    #   from innovations (predictions) to restore consistency.
    #
    # Limitation: only the diagonal is adapted (cross-covariances kept at zero),
    # which assumes sensor channels are independent. Full covariance adaptation
    # would require O(n^2) samples to avoid overfitting.

    def __init__(self, window=100):
        self.window = window               # Number of recent samples to keep
        # Sliding window of innovation vectors (z - H*x_pred) at each update step
        self._innovations = deque(maxlen=window)
        # Sliding window of residual vectors (z - H*x_upd) at each update step
        self._residuals = deque(maxlen=window)

    def update(self, innovation, residual):
        # Call every filter update with the latest innovation and residual vectors.
        self._innovations.append(innovation)
        self._residuals.append(residual)

    def estimate_q(self, dim=6):
        # Estimate the process noise covariance Q from recent innovations.
        # Returns an identity-scaled diagonal matrix.
        # Minimum 10 samples required; otherwise returns a small default Q.
        if len(self._innovations) < 10:
            return np.eye(dim) * 0.001
        innov = np.array(self._innovations)
        # Sample covariance of innovations: cov(z - H*x_pred)
        # Under correct Q/R, this should equal H*P*H^T + R.
        # If innovations are larger than expected, Q is probably too low.
        cov = np.cov(innov, rowvar=False)
        # Use mean diagonal covariance scaled by 0.5 as the Q magnitude.
        # This heuristic assumes cross-correlations are negligible.
        # Scale factor 0.5 is empirical: innovation cov captures both Q and R,
        # so we attribute roughly half to process noise (Q) and half to
        # measurement noise (R). More sophisticated methods use
        # autocovariance least-squares (ALS), but this is simpler and stable.
        Q = np.eye(dim) * np.mean(np.diag(cov)) * 0.5
        return Q

    def estimate_r(self, dim=6):
        # Estimate the measurement noise covariance R from recent residuals.
        # Residuals (z - H*x_upd) represent post-update errors and should
        # reflect measurement noise alone (process noise is already folded
        # into the update via K). If R is underestimated, post-fit residuals
        # will be larger than the filter expects.
        if len(self._residuals) < 10:
            return np.eye(dim) * 0.1
        resid = np.array(self._residuals)
        cov = np.cov(resid, rowvar=False)
        # Same 0.5 heuristic: residual covariance partially reflects
        # (I - H*K)*noise, so we halve to approximate true R.
        R = np.eye(dim) * np.mean(np.diag(cov)) * 0.5
        return R
