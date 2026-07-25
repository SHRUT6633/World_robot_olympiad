class AdaptivePID:
    def __init__(self, kp=0.5, ki=0.05, kd=0.01, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self._integral = 0.0
        self._last_error = 0.0
        self._last_output = 0.0

    def compute(self, error, limit=None):
        self._integral += error * self.dt
        derivative = (error - self._last_error) / self.dt
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._last_error = error
        if limit is not None:
            output = max(-limit, min(limit, output))
        self._last_output = output
        return output

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
