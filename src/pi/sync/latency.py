# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sync/latency.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Round-trip latency measurement
# =============================================================================

# =============================================================================
# latency.py — Round-trip latency measurement with sliding-window averaging
# =============================================================================
# Whenever a message is sent over the network (or inter-process), call
# record() with the send_time.  The class stores the last N samples in a
# deque and provides a running average via the avg_latency_ms property.
# =============================================================================

import time                                 # For perf_counter() — high-res timer
from collections import deque               # O(1) fixed-length sliding window
import numpy as np                          # Fast mean computation


class LatencyCompensator:
    """
    Sliding-window latency tracker.

    Typical usage
    -------------
        comp = LatencyCompensator(max_history=200)
        t0 = time.perf_counter()
        # ... send data ...
        # ... on the receiving side (or when reply arrives):
        comp.record("camera_frame", t0)
        print(f"Current avg latency: {comp.avg_latency_ms:.1f} ms")

    The average can be used by a prediction/filter module to dead-reckon
    sensor values while waiting for the actual data to arrive.
    """

    def __init__(self, max_history=100):
        # _history : fixed-size deque; older entries are evicted automatically.
        self._history = deque(maxlen=max_history)
        self._avg_latency = 0.0              # Cached mean (seconds)

    # ------------------------------------------------------------------
    # record — Log a single latency measurement
    # ------------------------------------------------------------------
    def record(self, name, send_time):
        """
        Measure the interval between *send_time* and now, store it, and
        recompute the running average.

        Parameters
        ----------
        name      : str   — Label for the measurement (logged but not stored).
        send_time : float — The perf_counter() value when the message was sent.

        Returns
        -------
        float
            The measured latency in **seconds**.

        What happens if you change max_history:
        - Smaller deque  → average responds faster but is noisier.
        - Larger deque   → average is smoother but lags behind real changes.
        """
        latency = time.perf_counter() - send_time
        self._history.append(latency)         # Append; auto-removes oldest if full
        self._avg_latency = np.mean(self._history)  # Recompute mean over window
        return latency

    # ------------------------------------------------------------------
    # avg_latency_ms — Running average, converted to milliseconds
    # ------------------------------------------------------------------
    @property
    def avg_latency_ms(self):
        """
        The cached average latency expressed in milliseconds.

        Returns
        -------
        float
            Average latency (ms).  Returns 0.0 if no calls to record() yet.
        """
        return self._avg_latency * 1000.0     # Convert seconds → milliseconds
