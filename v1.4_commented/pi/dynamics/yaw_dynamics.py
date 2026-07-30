import numpy as np


class YawDynamics:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.yaw_rate = 0.0

    def update(self, v, delta, wheelbase=0.26):
        if abs(v) < 0.01:
            self.yaw_rate = 0.0
        else:
            self.yaw_rate = (v / wheelbase) * np.tan(delta)
        return self.yaw_rate

    def integrate_yaw(self, yaw):
        return yaw + self.yaw_rate * self.dt
