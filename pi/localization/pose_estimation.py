import numpy as np


class PoseEstimator:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.yaw_rate = 0.0

    def update_odometry(self, v, yaw_rate, dt):
        self.x += v * np.cos(self.heading) * dt
        self.y += v * np.sin(self.heading) * dt
        self.heading += yaw_rate * dt
        self.velocity = v
        self.yaw_rate = yaw_rate

    def update_absolute(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def to_dict(self):
        return {
            "x": self.x, "y": self.y, "heading": self.heading,
            "v": self.velocity, "yaw_rate": self.yaw_rate,
        }
