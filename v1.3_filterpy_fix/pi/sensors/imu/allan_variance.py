import numpy as np
from ...system.logger import log


class AllanVariance:
    def __init__(self, max_samples=10000):
        self.max_samples = max_samples
        self._data = []

    def add_sample(self, value):
        self._data.append(value)
        if len(self._data) > self.max_samples:
            self._data.pop(0)

    def compute(self, log_spacing=True):
        if len(self._data) < 10:
            return {}, {}
        data = np.array(self._data)
        n = len(data)
        max_pow = int(np.log2(n))
        tau_values = [2 ** i for i in range(1, max_pow)]
        avar = {}
        for tau in tau_values:
            m = n // tau
            if m < 2:
                continue
            theta = np.mean(data[: m * tau].reshape(m, tau), axis=1)
            avar[tau] = 0.5 * np.mean(np.diff(theta) ** 2)
        return avar
