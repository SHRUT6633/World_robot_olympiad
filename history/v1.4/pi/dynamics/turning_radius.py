import numpy as np


class TurningRadiusPredictor:
    # Predicts the steady-state turning radius of the robot given a steering
    # angle, based on the kinematic bicycle model.
    # This is used for path planning and feedforward control to understand
    # how sharp a turn the robot will make.

    def __init__(self, wheelbase=0.26):
        # L: distance between front and rear axles (m).
        # Longer wheelbase = larger turning radius for the same steering angle
        # (less maneuverable but more stable at speed).
        self.L = wheelbase

    def predict(self, delta):
        # delta: steering angle (rad).
        # Returns: turning radius (m).
        #
        # Formula: R = |L / tan(delta)|.
        # This is the kinematic bicycle model: the rear axle follows a circle
        # of radius R when the front wheel is steered by delta.
        #
        # If delta = 0, returns infinity (straight ahead).
        # A small delta produces a very large radius (gentle curve).
        # A large delta produces a small radius (sharp turn).
        #
        # The 1e-6 threshold avoids division by zero from numerical noise.
        # Changing L scales the radius proportionally.
        if abs(delta) < 1e-6:
            return float("inf")
        return self.L / abs(np.tan(delta))

    def min_radius(self, max_steering):
        # max_steering: the maximum achievable steering angle (rad).
        # Returns: the minimum possible turning radius (m) at that steering limit.
        #
        # This tells you the tightest circle the robot can turn.
        # Used for path feasibility checking and to ensure planned paths
        # respect the robot's physical limits.
        return self.L / abs(np.tan(max_steering))
