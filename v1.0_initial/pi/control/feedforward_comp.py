import numpy as np


class FeedforwardCompensation:
    def __init__(self, mass=2.0, friction=0.1):
        self.mass = mass
        self.friction = friction

    def compute(self, target_accel, v):
        return self.mass * target_accel + self.friction * v
