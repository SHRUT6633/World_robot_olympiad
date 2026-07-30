import numpy as np
from ...system.logger import log


class AllanVariance:
    """
    Allan variance analysis for IMU noise characterisation.

    What is Allan variance?
      Allan variance (AVAR) is a time-domain analysis technique developed
      by David W. Allan to study the noise characteristics of precision
      oscillators. It has become the standard for MEMS IMU characterisation.

      Instead of computing the variance of all samples (which is not
      stationary for inertial sensors), AVAR computes the variance of
      cluster averages at different cluster times (τ).

      The AVAR plot (log τ vs. log σ(τ)) reveals:
        - Angle random walk (ARW) — slope -1/2: white noise on gyro.
        - Bias instability — minimum of the curve: the lowest drift level.
        - Rate random walk — slope +1/2: slowly varying bias.
        - Quantisation noise — slope -1.

    Why is this important for the robot?
      - ARW tells us how much the integrated heading drifts over short
        periods (seconds). Lower ARW = better dead-reckoning.
      - Bias instability tells us the minimum achievable heading drift
        over long periods (minutes). For a WRO match (~2 minutes), this
        is the dominant error source.
      - Knowledge of these parameters lets us tune the filter cutoffs
        and set appropriate thresholds for state estimation (e.g. EKF
        measurement noise covariances).

    How this code works:
      1. Collect many stationary IMU samples (e.g. 10,000 at 100 Hz = 100 s).
      2. Compute AVAR for τ = 2, 4, 8, 16, ... up to N/2.
      3. Each AVAR(τ) = 0.5 * mean of (difference of consecutive τ-cluster
         means)^2.

    Configuration:
      max_samples : Maximum number of samples to keep (default 10,000).
                    This controls the maximum τ we can analyse.

    Usage:
      After a data collection run, add_sample() repeatedly, then call
      compute() to get the AVAR dictionary.

      The robot is NOT doing this in real-time during the match — this
      is a characterisation tool used during development to tune filters
      and set EKF parameters.
    """

    def __init__(self, max_samples=10000):
        # Maximum samples to hold in the buffer (memory limit).
        self.max_samples = max_samples
        # List of accumulated samples (float values, e.g. gyro Z in °/s).
        self._data = []

    def add_sample(self, value):
        """
        Append a sensor sample to the dataset.

        If the buffer exceeds max_samples, the oldest sample is dropped
        (FIFO behaviour).
        """
        self._data.append(value)
        if len(self._data) > self.max_samples:
            self._data.pop(0)

    def compute(self, log_spacing=True):
        """
        Compute Allan variance for the accumulated data.

        log_spacing=True (default):
          Evaluates AVAR only at powers of 2 (τ = 2, 4, 8, ...) for
          computational efficiency. This is standard for characterisation
          plots (log-log axes).

        log_spacing=False:
          Would evaluate at every integer τ (more computationally expensive,
          not implemented for now).

        Returns:
          tau_values : list of int — cluster times used.
          avar       : dict {tau: variance_value} — Allan variance at each τ.

        If fewer than 10 samples are available, returns empty dicts.
        """
        if len(self._data) < 10:
            # Not enough data for meaningful analysis.
            return {}, {}
        data = np.array(self._data)
        n = len(data)
        max_pow = int(np.log2(n))
        # Tau values at powers of 2 (standard log-spaced analysis).
        tau_values = [2 ** i for i in range(1, max_pow)]
        avar = {}
        for tau in tau_values:
            m = n // tau  # Number of non-overlapping clusters.
            if m < 2:
                continue  # Need at least 2 clusters for variance.
            # Reshape into (m, τ) clusters and take mean of each.
            theta = np.mean(data[: m * tau].reshape(m, tau), axis=1)
            # Allan variance: 0.5 * mean of (consecutive difference)^2.
            avar[tau] = 0.5 * np.mean(np.diff(theta) ** 2)
        return avar
