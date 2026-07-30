import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerkedScaledSigmaPoints
from ..system.logger import log


class RobotUKF:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.dim_x = 6
        self.dim_z = 6

        points = MerkedScaledSigmaPoints(n=self.dim_x, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x, dim_z=self.dim_z, dt=dt, points=points,
            fx=self._fx, hx=self._hx
        )
        self.ukf.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ukf.P = np.eye(self.dim_x) * 0.1
        self.ukf.Q = np.eye(self.dim_x) * 0.001
        self.ukf.R = np.eye(self.dim_z) * 0.1

    @staticmethod
    def _fx(state, dt):
        x, y, heading, v, a, yaw_rate = state
        x += v * np.cos(heading) * dt
        y += v * np.sin(heading) * dt
        heading += yaw_rate * dt
        v += a * dt
        return np.array([x, y, heading, v, a, yaw_rate])

    @staticmethod
    def _hx(state):
        x, y, heading, v, a, yaw_rate = state
        return np.array([x, y, heading, v, a, yaw_rate])

    def predict(self, dt=None):
        if dt:
            self.ukf.dt = dt
        self.ukf.predict()

    def update(self, z):
        self.ukf.update(z)

    @property
    def state(self):
        return self.ukf.x.copy()

    @property
    def covariance(self):
        return self.ukf.P.copy()

    def set_process_noise(self, Q):
        self.ukf.Q = Q

    def set_measurement_noise(self, R):
        self.ukf.R = R

    def set_state(self, x, P=None):
        self.ukf.x = np.array(x)
        if P is not None:
            self.ukf.P = np.array(P)
