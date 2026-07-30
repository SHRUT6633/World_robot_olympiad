import numpy as np


class TireSlipEstimator:
    def __init__(self, mu=0.8):
        self.mu = mu

    def estimate_slip_angle(self, vy, vx, yaw_rate, wheelbase_rear=0.13):
        if abs(vx) < 0.01:
            return 0.0
        return np.arctan2(vy - wheelbase_rear * yaw_rate, vx)

    def lateral_force(self, slip_angle):
        return -self.mu * np.tanh(10 * slip_angle)
