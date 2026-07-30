class AntiWindup:
    def __init__(self, dt=0.01, clamp_min=-10.0, clamp_max=10.0):
        self.dt = dt
        self.clamp_min = clamp_min
        self.clamp_max = clamp_max

    def clamp(self, integral, error, output, limit):
        if output >= limit and error > 0:
            return integral
        if output <= -limit and error < 0:
            return integral
        return integral + error * self.dt

    def conditional(self, integral, error, output, limit):
        if output >= limit and error > 0:
            return integral
        if output <= -limit and error < 0:
            return integral
        return integral + error * self.dt

    def apply(self, integral, output):
        if output > self.clamp_max:
            return integral
        if output < self.clamp_min:
            return integral
        return integral
