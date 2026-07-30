from collections import deque
from ..system.logger import log


class SensorBufferManager:
    # ------------------------------------------------------------------
    # 1) Constructor: initialises the ring-buffer registry.
    #
    #    maxlen=100
    #      – Default capacity for every sensor buffer (can be overridden
    #        per-buffer via create()).
    #      – 100 samples @ 20 Hz = 5 seconds of history.
    #      – Larger maxlen = more memory, longer history; smaller = less
    #        memory but older samples are evicted sooner.
    #
    #    _buffers : dict[str, deque]
    #      – Maps sensor name (e.g. "camera", "lidar", "imu") to a
    #        double-ended queue that automatically drops the oldest
    #        element when full.
    # ------------------------------------------------------------------
    def __init__(self, maxlen=100):
        self._buffers = {}
        self.maxlen = maxlen

    # ------------------------------------------------------------------
    # 2) create(name, maxlen=None)
    #
    #    Creates a new named ring buffer.  If maxlen is not given, the
    #    default self.maxlen (100) is used.
    #
    #    Logging:
    #      Calls the project-level logger so developers can trace buffer
    #      creation events during debugging.
    # ------------------------------------------------------------------
    def create(self, name, maxlen=None):
        self._buffers[name] = deque(maxlen=maxlen or self.maxlen)

    # ------------------------------------------------------------------
    # 3) push(name, data)
    #
    #    Appends a single data sample to the named buffer.
    #    If the buffer does not yet exist, it is auto-created with the
    #    default maxlen.
    #
    #    Because deque is fixed-size, appending when full silently drops
    #    the oldest sample – perfect for real-time sensor streams.
    # ------------------------------------------------------------------
    def push(self, name, data):
        if name not in self._buffers:
            self.create(name)
        self._buffers[name].append(data)

    # ------------------------------------------------------------------
    # 4) latest(name) -> data | None
    #
    #    Returns the most recent sample from buffer `name`, or None if
    #    the buffer does not exist (e.g. sensor never pushed).
    #
    #    Used by the planning and control loops to get the freshest
    #    sensor reading without blocking.
    # ------------------------------------------------------------------
    def latest(self, name):
        buf = self._buffers.get(name)
        return buf[-1] if buf else None

    # ------------------------------------------------------------------
    # 5) all_latest() -> dict[str, data | None]
    #
    #    Convenience method that returns *all* buffers' latest samples
    #    in one dictionary.  Useful for a "sensor snapshot" at the start
    #    of a planning tick.
    # ------------------------------------------------------------------
    def all_latest(self):
        return {n: buf[-1] if buf else None for n, buf in self._buffers.items()}

    # ------------------------------------------------------------------
    # 6) clear(name=None)
    #
    #    If a name is given, only that buffer is cleared.
    #    Otherwise, every buffer is emptied.
    #
    #    Behaviour if name does not exist:
    #      A new empty deque is created (via .get()) and immediately
    #      cleared – effectively a no-op.
    #
    #    Typical use:
    #      Called on robot startup or after a mode switch to flush stale
    #      sensor history.
    # ------------------------------------------------------------------
    def clear(self, name=None):
        if name:
            self._buffers.get(name, deque()).clear()
        else:
            for b in self._buffers.values():
                b.clear()

    # ------------------------------------------------------------------
    # 7) __getitem__(name) -> list
    #
    #    Allows bracket-indexed access:
    #        buffer["camera"]  ->  list of all camera samples
    #
    #    Returns a *copy* of the deque as a plain Python list so the
    #    caller cannot accidentally mutate the internal buffer.
    #    If the buffer does not exist, an empty list is returned.
    #
    #    Connection to the system:
    #      - Used by analysis / debugging tools and by
    #        trajectory-generation code when it needs more than just
    #        the latest sample (e.g. to compute velocities via finite
    #        differences over the last N poses).
    # ------------------------------------------------------------------
    def __getitem__(self, name):
        return list(self._buffers.get(name, []))
