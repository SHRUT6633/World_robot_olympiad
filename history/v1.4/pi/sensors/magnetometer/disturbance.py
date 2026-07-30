import numpy as np
from ...system.logger import log


class MagneticDisturbanceDetector:
    """
    Detects transient magnetic disturbances that corrupt compass readings.

    Why this matters:
      Magnetometers are sensitive to external magnetic fields. In a WRO
      environment, the robot may encounter:
        - Magnetic fields from other robots' motors (high-current PWM).
        - Metal structures in the field (goal posts, elevated bridges).
        - Electromagnetic interference from power cables under the floor.
        - The robot's own motors during high-current acceleration.

      When a disturbance is detected, the navigation system should ignore
      or down-weight the magnetometer heading and rely on the gyroscope
      (dead-reckoning) until the disturbance passes.

    Detection method:
      1. Compute the magnitude of the 3-axis magnetic vector:
           |B| = sqrt(Bx² + By² + Bz²)
      2. Maintain a sliding window of recent magnitudes.
      3. A reading is considered "disturbed" if:
           |magnitude - window_mean| > threshold * window_std

      If the standard deviation is small (stable field), even a small
      absolute disturbance triggers detection.
      If the standard deviation is large (sensor is already disturbed or
      being calibrated), the threshold scales up — preventing false
      positives.

    Configuration:
      threshold : Number of standard deviations for the outlier threshold.
                 Default 100. NOTE: This seems very high. Typical z-score
                 thresholds are 2–5. A value of 100 means the detector will
                 rarely trigger. Consider lowering to ~3–5 for effective detection.
      window    : Number of recent magnitude samples to keep.

    Example:
      In a clean environment, the field magnitude might be ~0.45 gauss
      with σ ≈ 0.005 gauss. If a motor starts and injects 0.1 gauss of
      interference, the z-score = 0.1 / 0.005 = 20. With threshold=100,
      this would NOT be detected! With threshold=3, it would be.

    How the robot uses this:
      The navigation system calls detect(mag) on each iteration.
      If True is returned, the magnetometer-based heading is not trusted,
      and the EKF / complementary filter relies solely on gyro integration
      until the disturbance clears (detect returns False again).

    Note:
      The baseline is NOT explicitly stored; it is computed as the
      windowed mean. This means slow changes (e.g. field change from
      position A to B on the field) are NOT falsely detected after the
      window adapts (~10 samples = 0.1 s at 100 Hz).
    """

    def __init__(self, threshold=100, window=10):
        # Z-score threshold for disturbance detection.
        # WARNING: default 100 is likely a bug — should be ~3–5.
        self.threshold = threshold
        # Sliding window size (number of magnitude samples).
        self.window = window
        # Buffer of recent magnetic field magnitudes.
        self._history = []
        # Persistent baseline (not used currently; computed from window).
        self._baseline = None

    def detect(self, mag):
        """
        Check whether the current magnetic reading is disturbed.

        mag : numpy array (3,) — magnetic field vector in sensor counts
              or gauss (units don't matter as long as consistent).

        Returns:
          True if the reading is likely disturbed (outlier).
          False if the reading is consistent with recent history.

        The first 5 readings are always accepted (not enough history).
        """
        magnitude = np.linalg.norm(mag)
        self._history.append(magnitude)
        # Maintain FIFO buffer.
        if len(self._history) > self.window:
            self._history.pop(0)
        # Need at least 5 samples for a meaningful baseline.
        if len(self._history) < 5:
            return False
        mean = np.mean(self._history)
        std = np.std(self._history) + 1e-6
        disturbed = abs(magnitude - mean) > self.threshold * std
        return bool(disturbed)
