import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from ..system.logger import log


class RobotEKF(ExtendedKalmanFilter):
    def __init__(self, dt=0.01):
        super().__init__(dim_x=6, dim_z=6)
        self.dt = dt
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(6) * 0.1
        self.Q = np.eye(6) * 0.001
        self.R = np.eye(6) * 0.1
        self.F = np.eye(6)
        self.H = np.eye(6)

    def predict(self, dt=None):
        dt = dt or self.dt
        x, y, heading, v, a, yaw_rate = self.x
        self.F = np.array([
            [1, 0, -v * np.sin(heading) * dt, np.cos(heading) * dt, 0, 0],
            [0, 1,  v * np.cos(heading) * dt, np.sin(heading) * dt, 0, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, dt, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        self.x[0] += v * np.cos(heading) * dt
        self.x[1] += v * np.sin(heading) * dt
        self.x[2] += yaw_rate * dt
        self.x[3] += a * dt
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
