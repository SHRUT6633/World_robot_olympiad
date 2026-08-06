# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/base.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Abstract base class for all robot sensors
# =============================================================================

import abc
import time
try:
    import numpy as np
except ImportError:
    np = None
from ..system.logger import log


class SensorBase(abc.ABC):
    """
    Abstract base class for ALL sensors on the robot.

    Every sensor (ToF, IMU, magnetometer, camera) inherits from this class.
    It provides a uniform interface so the robot's control system can treat
    every sensor the same way: init(), read(), read_raw(), and close().

    The _read_interval allows a rate-limiter to be added later: if the
    control loop calls read() faster than this interval, stale data can
    be returned (not yet implemented — here for future use).
    """

    def __init__(self, name: str):
        # Human-readable sensor name (e.g. "VL53L0X_front", "MPU6050").
        self.name = name
        # Master enable/disable flag. When False, read() returns None.
        self._enabled = True
        # Timestamp (seconds) of the last successful read. Used for rate limiting.
        self._last_read = 0.0
        # Minimum time (seconds) between two hardware reads. 10 ms = 100 Hz.
        self._read_interval = 0.01
        # Rate-limited error logging: logs at most once every 2 seconds
        # and auto-disables sensor after 50 consecutive failures.
        self._last_error_log = 0.0
        self._error_count = 0
        self._error_cooldown = 2.0
        self._max_errors = 50

    def _log_error(self, msg):
        now = time.monotonic()
        self._error_count += 1
        # Auto-disable sensor after max_errors consecutive failures to prevent
        # the control loop from acting on stale/bogus data.
        if self._error_count >= self._max_errors:
            self._enabled = False
        # Rate-limit error logs to once per cooldown period (2 s default)
        # to avoid flooding the log with repeated identical errors.
        if now - self._last_error_log >= self._error_cooldown:
            self._last_error_log = now
            log.warn(f"{self.name}: {msg}")

    @abc.abstractmethod
    def init(self):
        """
        One-time hardware initialisation.

        Opens the I2C bus, configures sensor registers (range, filters,
        power mode), and sets up GPIO pins (e.g. XSHUT for multi-sensor
        addressing). Must be called once before any read().
        """

    @abc.abstractmethod
    def read_raw(self):
        """
        Read the sensor's raw, unfiltered data from hardware.

        Returns sensor-specific values (int, float, numpy array, or dict).
        No software filtering is applied — this is the closest to the
        physical measurement. Used internally by read().
        """

    def read(self):
        """
        High-level read that respects the enabled flag.

        Returns None if the sensor is disabled (self._enabled == False),
        otherwise delegates to read_raw(). Subclasses like VL53L0X override
        this to add filtering (median, moving average, outlier rejection).
        """
        if not self._enabled:
            return None
        return self.read_raw()

    def close(self):
        """
        Release hardware resources.

        Closes the I2C bus (smbus2.SMBus.close()) and any GPIO pins.
        Called during robot shutdown so the bus is free for other processes.
        """

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, val: bool):
        self._enabled = val


class FilteredSensorMixin:
    """
    Mixin that adds software filtering to any sensor reading.

    Three filter strategies are available:
      - median:       Robust against single spike outliers.
      - moving avg:   Smooths noise (low-pass) but responds slower.
      - outlier:      Rejects values >3 sigma from running mean.

    The window_size controls the memory length: larger = smoother but
    laggier response. For a robot moving at ~1 m/s, a window of 5–10
    samples at 50 Hz gives ~100–200 ms of lag — acceptable for obstacle
    avoidance but not for crash detection.

    Internally maintains a FIFO buffer per axis (one buffer shared across
    all filter methods — beware of mixing calls).
    """

    def __init__(self, window_size=5):
        # Number of past samples kept in the sliding window.
        self.window_size = window_size
        # FIFO buffer: appends newest, pops oldest when > window_size.
        self._buffer = []

    def filter_median(self, value):
        # Median filter: robust to single-pulse outliers (e.g. ToF glint).
        # Adds overhead due to sorting O(N log N) but rejects spikes cleanly.
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(np.median(self._buffer))

    def filter_moving_avg(self, value):
        # Simple FIR low-pass: attenuates high-frequency sensor noise but
        # introduces (window_size - 1)/2 samples of phase delay.
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(np.mean(self._buffer))

    def filter_outliers(self, value, threshold=3.0):
        # Need at least 3 samples for meaningful mean/std statistics.
        if len(self._buffer) < 3:
            self._buffer.append(value)
            return value
        mu = np.mean(self._buffer)
        sigma = np.std(self._buffer) + 1e-6
        # Z-score outlier rejection: if |z| > threshold, hold last mean
        # instead of letting the spike corrupt the control loop.
        if abs(value - mu) > threshold * sigma:
            return float(mu)
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(value)
