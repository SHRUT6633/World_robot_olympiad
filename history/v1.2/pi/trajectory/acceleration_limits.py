import numpy as np


class AccelerationLimiter:
    def __init__(self, max_accel=2.0, max_decel=3.0):
        self.max_a = max_accel
        self.max_d = max_decel

    def limit(self, current_v, target_v, dt):
        dv = target_v - current_v
        if dv > 0:
            dv = min(dv, self.max_a * dt)
        else:
            dv = max(dv, -self.max_d * dt)
        return current_v + dv
