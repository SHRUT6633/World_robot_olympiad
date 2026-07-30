import numpy as np


class PoseEstimator:
    # Simple dead-reckoning pose estimator using odometry updates.
    #
    # This is the lowest-level pose tracker. It can be updated in two ways:
    #   1. update_odometry(v, yaw_rate, dt) — integrates wheel odometry forward in time.
    #   2. update_absolute(x, y, heading) — overwrites pose from an external source
    #      (e.g., filter output, GPS, visual localisation).
    #
    # The pose is represented in a global 2D Cartesian frame:
    #   x, y: position in meters
    #   heading: orientation in radians (0 = +x axis, increasing CCW)
    #   velocity: forward speed in m/s
    #   yaw_rate: angular velocity in rad/s

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.yaw_rate = 0.0

    def update_odometry(self, v, yaw_rate, dt):
        # Integrate odometry using the bicycle model:
        #   dx = v * cos(heading) * dt
        #   dy = v * sin(heading) * dt
        #   dheading = yaw_rate * dt
        self.x += v * np.cos(self.heading) * dt
        self.y += v * np.sin(self.heading) * dt
        self.heading += yaw_rate * dt
        self.velocity = v
        self.yaw_rate = yaw_rate

    def update_absolute(self, x, y, heading):
        # Hard-set the pose from an external filtered/sensor estimate.
        # This overwrites any accumulated dead-reckoning drift.
        self.x = x
        self.y = y
        self.heading = heading

    def to_dict(self):
        # Returns pose as a dictionary for easy serialisation/logging.
        return {
            "x": self.x, "y": self.y, "heading": self.heading,
            "v": self.velocity, "yaw_rate": self.yaw_rate,
        }
