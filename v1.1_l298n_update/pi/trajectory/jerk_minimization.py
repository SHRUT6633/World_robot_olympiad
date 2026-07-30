import numpy as np


class JerkMinimizer:
    def __init__(self, max_jerk=5.0):
        self.max_jerk = max_jerk
        self._last_jerk = 0.0

    def limit(self, acceleration, dt):
        jerk = (acceleration - self._last_jerk) / dt
        jerk = np.clip(jerk, -self.max_jerk, self.max_jerk)
        accel = self._last_jerk + jerk * dt
        self._last_jerk = jerk
        return accel
