import time


class ServoPID:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.kp = 1.5
        self.ki = 0.3
        self.kd = 0.3

        self._integral = 0.0
        self._last_error = 0.0
        self._last_position = 0.0
        self._last_output = 90

        self.min_angle = -30.0
        self.max_angle = 30.0
        self.min_pulse = 1000
        self.max_pulse = 2000

        self.lp_alpha = 0.3

    def lowpass(self, raw, prev):
        return self.lp_alpha * raw + (1 - self.lp_alpha) * prev

    def compute(self, target_deg, current_deg):
        current_filtered = self.lowpass(current_deg, self._last_position)

        error = target_deg - current_filtered
        self._integral += error * self.dt
        derivative = (error - self._last_error) / self.dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._last_error = error
        self._last_position = current_filtered

        pulse = 1500 + int(output * (self.max_pulse - 1500) / self.max_angle)
        pulse = max(self.min_pulse, min(self.max_pulse, pulse))
        self._last_output = pulse
        return pulse

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_position = 0.0
