import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints


def fx(state, dt, **kwargs):
    x, y, heading, speed, accel, yaw_rate = state
    heading += yaw_rate * dt
    heading = np.arctan2(np.sin(heading), np.cos(heading))
    speed += accel * dt
    x += speed * np.cos(heading) * dt
    y += speed * np.sin(heading) * dt
    return np.array([x, y, heading, speed, accel, yaw_rate])


def hx(state):
    x, y, heading, speed, accel, yaw_rate = state
    return np.array([x, y, heading])


class UKF:
    def __init__(self):
        points = MerweScaledSigmaPoints(6, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=0.02,
                                          fx=fx, hx=hx, points=points)
        self.ukf.x = np.zeros(6)
        self.ukf.P = np.eye(6) * 0.1
        self.ukf.Q = np.eye(6) * 1e-3
        self.ukf.R = np.eye(3) * 1e-1

    def predict(self, dt: float) -> None:
        self.ukf.dt = dt
        self.ukf.predict()

    def correct(self, z: np.ndarray) -> None:
        self.ukf.update(z)

    def pose(self) -> tuple:
        return tuple(self.ukf.x[:3])
