import time
from collections import deque
import numpy as np


class LatencyCompensator:
    def __init__(self, max_history=100):
        self._history = deque(maxlen=max_history)
        self._avg_latency = 0.0

    def record(self, name, send_time):
        latency = time.perf_counter() - send_time
        self._history.append(latency)
        self._avg_latency = np.mean(self._history)
        return latency

    @property
    def avg_latency_ms(self):
        return self._avg_latency * 1000.0
