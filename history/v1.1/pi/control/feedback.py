import numpy as np


class FeedbackSteering:
    def __init__(self, kp=1.0, kd=0.1):
        self.kp = kp
        self.kd = kd
        self._last_error = 0.0

    def compute(self, heading_error):
        derivative = heading_error - self._last_error
        self._last_error = heading_error
        return self.kp * heading_error + self.kd * derivative
