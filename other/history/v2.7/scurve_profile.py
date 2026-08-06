import math

class SCurveProfile:
    def __init__(self, ramp_time=0.5):
        self.ramp_time = ramp_time

    @staticmethod
    def smoothstep(t):
        if t <= 0.0:
            return 0.0
        if t >= 1.0:
            return 1.0
        return t * t * (3.0 - 2.0 * t)

    def velocity(self, t, v0, v1):
        tau = t / self.ramp_time if self.ramp_time > 0 else 1.0
        fraction = self.smoothstep(tau)
        return v0 + (v1 - v0) * fraction

    def acceleration(self, t, v0, v1):
        dv = v1 - v0
        tau = t / self.ramp_time if self.ramp_time > 0 else 1.0
        if tau <= 0.0 or tau >= 1.0:
            return 0.0
        da = 6.0 * tau * (1.0 - tau) * dv / (self.ramp_time * self.ramp_time)
        return da

    def plan(self, v0, v1, steps=50):
        table = []
        for i in range(steps + 1):
            t = (i / steps) * self.ramp_time
            v = self.velocity(t, v0, v1)
            table.append((t, v))
        return table
