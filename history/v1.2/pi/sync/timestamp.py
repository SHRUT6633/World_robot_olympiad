import time


class TimestampSync:
    def __init__(self):
        self._epoch = time.perf_counter()

    def now(self):
        return time.perf_counter() - self._epoch

    def stamp(self):
        return {"ts": self.now(), "monotonic": time.perf_counter()}


class SensorTimestamp:
    def __init__(self, name):
        self.name = name
        self.sync = TimestampSync()
        self._last_ts = 0.0

    def stamp(self):
        self._last_ts = self.sync.now()
        return self._last_ts
