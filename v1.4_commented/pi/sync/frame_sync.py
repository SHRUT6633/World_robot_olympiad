import time
import threading
from collections import deque
from ..system.logger import log


class FrameSync:
    def __init__(self, tolerance_ms=5):
        self.tolerance = tolerance_ms / 1000.0
        self._buffers = {}
        self._lock = threading.Lock()

    def register(self, name, buffer_size=2):
        with self._lock:
            self._buffers[name] = deque(maxlen=buffer_size)

    def push(self, name, data):
        with self._lock:
            if name in self._buffers:
                self._buffers[name].append((time.perf_counter(), data))

    def synchronize(self):
        with self._lock:
            if len(self._buffers) < 2:
                return None
            ref = None
            for name, buf in self._buffers.items():
                if not buf:
                    continue
                ts, data = buf[-1]
                if ref is None or ts > ref[0]:
                    ref = (ts, name, data)
            if ref is None:
                return None
            synced = {"_ref_ts": ref[0]}
            for name, buf in self._buffers.items():
                best = None
                for ts, data in buf:
                    dt = abs(ts - ref[0])
                    if dt <= self.tolerance:
                        if best is None or dt < best[0]:
                            best = (dt, data, ts)
                synced[name] = best[1] if best else None
            return synced
