# =============================================================================
# frame_sync.py — Multi-sensor frame synchronisation via timestamp alignment
# =============================================================================
# The robot may have several sensors (cameras, LIDAR, etc.) each producing
# data at slightly different rates or times.  FrameSync collects their most
# recent samples and, when synchronize() is called, finds the "reference"
# frame (the newest one) and then matches every other sensor's buffered
# samples that fall within a tolerance window.
# =============================================================================

import time                                 # For perf_counter() timestamps
import threading                            # Thread-safe buffer access
from collections import deque               # Fixed-length ring buffer per sensor
from ..system.logger import log             # Project logger (relative import)


class FrameSync:
    """
    Time-based hardware-frame synchroniser.

    Example
    -------
        sync = FrameSync(tolerance_ms=10)
        sync.register("camera_left", buffer_size=3)
        sync.register("lidar",       buffer_size=3)

        # ... in the sensor callbacks (possibly different threads) ...
        sync.push("camera_left", left_img)
        sync.push("lidar",       point_cloud)

        # ... in the main loop ...
        aligned = sync.synchronize()
        if aligned:
            process(aligned["camera_left"], aligned["lidar"])

    Thread safety
    -------------
    All public methods acquire a reentrant lock so they can be called from
    different sensor threads without corruption.
    """

    def __init__(self, tolerance_ms=5):
        # tolerance : maximum allowed timestamp difference (seconds).
        # Sensors whose newest sample differs by more than this will yield
        # None for that sensor cycle.
        self.tolerance = tolerance_ms / 1000.0

        self._buffers = {}                   # name → deque of (ts, data)
        self._lock = threading.Lock()        # Guards _buffers

    # ------------------------------------------------------------------
    # register — Add a sensor input to the syncer
    # ------------------------------------------------------------------
    def register(self, name, buffer_size=2):
        """
        Declare a sensor named *name* with a ring buffer of *buffer_size*.

        Larger buffer sizes help when sensors run at different rates — the
        syncer can look back further in time to find a match — but consume
        more memory.
        """
        with self._lock:
            self._buffers[name] = deque(maxlen=buffer_size)

    # ------------------------------------------------------------------
    # push — Submit a new data sample from a sensor
    # ------------------------------------------------------------------
    def push(self, name, data):
        """
        Append *(current_time, data)* to the named sensor's buffer.

        If *name* hasn't been registered, the call is silently ignored.
        """
        with self._lock:
            if name in self._buffers:
                self._buffers[name].append((time.perf_counter(), data))

    # ------------------------------------------------------------------
    # synchronize — Produce a time-aligned snapshot of all sensors
    # ------------------------------------------------------------------
    def synchronize(self):
        """
        Find the newest sample across all sensors (the *reference*), then
        for every other sensor pick the buffered sample whose timestamp is
        closest to that reference **and** within *tolerance*.

        Returns
        -------
        dict or None
            None if fewer than 2 sensors are registered or all buffers empty.
            Otherwise a dict:
                {"_ref_ts": <float>,
                 "sensor_a": <data or None>,
                 "sensor_b": <data or None>,
                 ...}
            A sensor entry is None when no buffer entry falls within the
            tolerance window of the reference.

        Effect of changing tolerance:
        - Smaller tolerance → stricter alignment; more None results.
        - Larger tolerance  → more frames pass but temporal misalignment grows.
        """
        with self._lock:
            # Need at least two sensors to synchronise anything
            if len(self._buffers) < 2:
                return None

            # ---- Step 1: pick the **newest** sample as the reference ----
            ref = None                        # (ts, name, data)
            for name, buf in self._buffers.items():
                if not buf:
                    continue                  # Empty buffer — skip
                ts, data = buf[-1]            # Most recent element in deque
                if ref is None or ts > ref[0]:
                    ref = (ts, name, data)

            if ref is None:                   # All buffers were empty
                return None

            # ---- Step 2: build the result dict ----
            synced = {"_ref_ts": ref[0]}      # Store the reference timestamp

            for name, buf in self._buffers.items():
                best = None                   # (dt, data, ts)
                for ts, data in buf:
                    dt = abs(ts - ref[0])     # Absolute time difference
                    if dt <= self.tolerance:  # Candidate is within the window
                        if best is None or dt < best[0]:
                            best = (dt, data, ts)

                # Assign the matched data (or None if nothing was in range)
                synced[name] = best[1] if best else None

            return synced
