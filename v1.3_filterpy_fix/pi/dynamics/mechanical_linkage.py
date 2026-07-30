import numpy as np


class MechanicalLinkage:
    def __init__(self, gear_ratio=1.0, max_steering_deg=30):
        self.gear_ratio = gear_ratio
        self.max_steering = np.radians(max_steering_deg)

    def servo_to_wheel(self, servo_angle):
        wheel_angle = servo_angle * self.gear_ratio
        return np.clip(wheel_angle, -self.max_steering, self.max_steering)

    def wheel_to_servo(self, wheel_angle):
        return wheel_angle / self.gear_ratio
