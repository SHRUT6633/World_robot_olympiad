import numpy as np


class EKF:
    def __init__(self):
        self.x = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.Q = np.diag([0.05, 0.05, np.radians(2.0)])
        self.R = np.diag([0.1, 0.1])

    def predict(self, v: float, omega: float, dt: float) -> None:
        theta = self.x[2]
        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt
        self.x[2] += omega * dt
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

        F = np.eye(3)
        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        self.P = F @ self.P @ F.T + self.Q

    def correct(self, z: np.ndarray) -> None:
        H = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P
