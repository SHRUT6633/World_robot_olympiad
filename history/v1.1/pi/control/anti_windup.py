class AntiWindup:
    def __init__(self, clamp_min=-10, clamp_max=10):
        self.clamp_min = clamp_min
        self.clamp_max = clamp_max

    def apply(self, integral, output):
        if output > self.clamp_max:
            return integral
        if output < self.clamp_min:
            return integral
        return integral

    @staticmethod
    def conditional(integral, error, output, limit):
        if output >= limit and error > 0:
            return integral
        if output <= -limit and error < 0:
            return integral
        return integral
