import time


class MotorPID:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.kp = 1.2
        self.ki = 0.1
        self._integral = 0.0
        self._last_error = 0.0
        self._last_speed = 0
        self.max_pwm = 255

    def _select_gains(self, target):
        if target < 0.3:
            kp = 0.3
        elif target < 0.8:
            kp = 0.6
        elif target < 1.5:
            kp = 0.9
        else:
            kp = 1.0
        return kp, self.ki

    def compute(self, target, current):
        if abs(target - current) < 0.02:
            return self._last_speed

        kp, ki = self._select_gains(target)
        error = target - current
        self._integral += error * self.dt
        derivative = (error - self._last_error) / self.dt

        output = kp * error + ki * self._integral
        self._last_error = error

        speed = self._last_speed + output * self.dt
        speed = max(0, min(self.max_pwm, speed))
        self._last_speed = int(speed)
        return self._last_speed

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_speed = 0

    def set_speed(self, target_ms, speed_ms):
        return self.compute(target_ms, speed_ms)
