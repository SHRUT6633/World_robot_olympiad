# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sync/timestamp.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Monotonic-clock timestamp synchronisation
# =============================================================================

# =============================================================================
# timestamp.py — Monotonic-clock timestamp synchronisation
# =============================================================================
# Provides a lightweight epoch-relative clock based on time.perf_counter()
# (which is immune to system-clock adjustments).  Two classes are exposed:
#
#   TimestampSync    — A global reference clock for the process.
#   SensorTimestamp  — Per-sensor wrapper that tracks the *last* stamp.
#
# All timestamps in the robot system should be relative to the same
# TimestampSync instance to enable cross-sensor alignment.
# =============================================================================

import time                                 # Standard-library time functions


class TimestampSync:
    """
    Monotonic clock with an arbitrary zero point (process start by default).

    Why monotonic?
        time.perf_counter() is not affected by NTP, user changes, or leap
        seconds, so it is safe for measuring intervals and ordering events.

    Attributes
    ----------
    _epoch : float
        The perf_counter() value captured at construction; all returned
        timestamps are relative to this.
    """

    def __init__(self):
        # Record the current perf_counter() value as the "epoch".
        # Every future call to now() will subtract this, yielding seconds
        # since this TimestampSync was created.
        self._epoch = time.perf_counter()

    # ------------------------------------------------------------------
    # now — Seconds since this object was created
    # ------------------------------------------------------------------
    def now(self):
        """
        Return time (seconds) elapsed since __init__.

        Returns
        -------
        float
            Monotonic offset from epoch.

        Example
        -------
        >>> ts = TimestampSync()
        >>> time.sleep(1)
        >>> ts.now()
        1.0001   # approximately
        """
        return time.perf_counter() - self._epoch

    # ------------------------------------------------------------------
    # stamp — Return a fully-qualified timestamp dict
    # ------------------------------------------------------------------
    def stamp(self):
        """
        Dict with the relative timestamp AND the raw perf_counter() value.

        The 'monotonic' field is useful when you need to reconstruct the
        absolute ordering across restarts (e.g. when logging to disk).

        Returns
        -------
        dict
            {"ts": <offset from epoch>, "monotonic": <raw perf_counter>}
        """
        return {"ts": self.now(), "monotonic": time.perf_counter()}


class SensorTimestamp:
    """
    Per-sensor timestamp wrapper.

    Each sensor (camera, LIDAR, IMU, …) should hold its own instance so
    that the system knows which reading was the most recent *for that sensor*.

    Attributes
    ----------
    name      : str       — Sensor identifier (e.g. "camera_left").
    sync      : TimestampSync — Shared reference clock.
    _last_ts  : float     — The last timestamp produced by stamp().
    """

    def __init__(self, name):
        self.name = name                      # Human-readable sensor label
        self.sync = TimestampSync()           # Each sensor gets its own reference
        self._last_ts = 0.0                   # No stamps yet

    # ------------------------------------------------------------------
    # stamp — Record-and-return a new timestamp
    # ------------------------------------------------------------------
    def stamp(self):
        """
        Update _last_ts to the current clock value and return it.

        Returns
        -------
        float
            Seconds since this sensor's own reference epoch.
        """
        self._last_ts = self.sync.now()       # Snapshot the current time
        return self._last_ts                  # Also return it for convenience
