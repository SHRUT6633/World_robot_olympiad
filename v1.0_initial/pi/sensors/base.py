import abc
import time
try:
    import numpy as np
except ImportError:
    np = None

class SensorBase(abc.ABC):
    def __init__(self, name: str):
        self.name = name
        self._enabled = True
        self._last_read = 0.0
        self._read_interval = 0.01

    @abc.abstractmethod
    def init(self):
        pass

    @abc.abstractmethod
    def read_raw(self):
        pass

    def read(self):
        if not self._enabled:
            return None
        return self.read_raw()

    def close(self):
        pass

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, val: bool):
        self._enabled = val


class FilteredSensorMixin:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self._buffer = []

    def filter_median(self, value):
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(np.median(self._buffer))

    def filter_moving_avg(self, value):
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(np.mean(self._buffer))

    def filter_outliers(self, value, threshold=3.0):
        if len(self._buffer) < 3:
            self._buffer.append(value)
            return value
        mu = np.mean(self._buffer)
        sigma = np.std(self._buffer) + 1e-6
        if abs(value - mu) > threshold * sigma:
            return float(mu)
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return float(value)
