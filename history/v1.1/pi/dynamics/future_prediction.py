import numpy as np


class FuturePositionPredictor:
    def __init__(self, horizon_s=1.0, steps=10):
        self.horizon = horizon_s
        self.steps = steps

    def predict(self, x, y, heading, v, delta, wheelbase=0.26):
        dt = self.horizon / self.steps
        trajectory = []
        cx, cy, ch = x, y, heading
        for _ in range(self.steps):
            cx += v * np.cos(ch) * dt
            cy += v * np.sin(ch) * dt
            ch += (v / wheelbase) * np.tan(delta) * dt
            trajectory.append((cx, cy))
        return np.array(trajectory)
