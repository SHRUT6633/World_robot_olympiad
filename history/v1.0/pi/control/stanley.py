import numpy as np


class StanleyController:
    def __init__(self, k=0.5, k_soft=1.0, max_steering=np.radians(30)):
        self.k = k
        self.k_soft = k_soft
        self.max_steering = max_steering

    def compute(self, robot_x, robot_y, robot_heading, target_x, target_y, target_heading, v):
        dx = target_x - robot_x
        dy = target_y - robot_y
        crosstrack = -np.sin(robot_heading) * dx + np.cos(robot_heading) * dy
        heading_error = target_heading - robot_heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        steer = heading_error + np.arctan2(self.k * crosstrack, self.k_soft + v)
        return np.clip(steer, -self.max_steering, self.max_steering)
