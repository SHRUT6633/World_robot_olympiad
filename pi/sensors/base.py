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
        if self._error_count >= self._max_errors:
            self._enabled = False
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
        """
        Replace value with the median of the last N samples.

        Median is very robust against outliers (e.g. a single ToF
        reading that bounces off a glossy surface). Slower than moving
        average due to sorting.
        """
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(np.median(self._buffer))

    def filter_moving_avg(self, value):
        """
        Replace value with the arithmetic mean of the last N samples.

        Equivalent to a simple low-pass FIR filter. Attenuates high-
        frequency sensor noise but introduces a phase delay equal to
        (window_size - 1) / 2 samples.
        """
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(np.mean(self._buffer))

    def filter_outliers(self, value, threshold=3.0):
        """
        Reject a value if it deviates more than `threshold` standard
        deviations from the running mean.

        If rejected, returns the current mean instead (hold-last-good).
        This prevents spurious readings (e.g. ultrasonic cross-talk,
        IR interference from sunlight) from corrupting the control loop.
        The 1e-6 epsilon prevents division-by-zero when buffer is constant.
        """
        if len(self._buffer) < 3:
            # Not enough history to compute statistics; accept blindly.
            self._buffer.append(value)
            return value
        mu = np.mean(self._buffer)
        sigma = np.std(self._buffer) + 1e-6
        if abs(value - mu) > threshold * sigma:
            # Outlier detected: return the mean and DO NOT add value to buffer.
            return float(mu)
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(value)
