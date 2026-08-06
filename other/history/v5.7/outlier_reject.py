import math
import numpy as np
from scipy.stats import chi2
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

CHI2_THRESHOLD = chi2.ppf(0.95, df=3)


class OutlierRejectUKF:
    def __init__(self):
        points = MerweScaledSigmaPoints(6, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=6, dim_z=3, dt=0.02, fx=self._fx, hx=self._hx, points=points
        )
        self.ukf.x = np.zeros(6)
        self.ukf.P = np.eye(6) * 0.1
        self.ukf.Q = np.eye(6) * 1e-3
        self._R_nominal = np.eye(3) * 1e-1
        self.ukf.R = self._R_nominal.copy()

    def predict(self, dt: float) -> None:
        self.ukf.dt = dt
        self.ukf.predict()

    def correct(self, z: np.ndarray) -> None:
        h = self._hx(self.ukf.x)
        y = z - h
        H = np.eye(3, 6)
        S = H @ self.ukf.P @ H.T + self.ukf.R
        try:
            d2 = y @ np.linalg.solve(S, y)
        except np.linalg.LinAlgError:
            d2 = 0.0
        if d2 > CHI2_THRESHOLD:
            weight = math.exp(-0.5 * (d2 - CHI2_THRESHOLD))
            self.ukf.R = self._R_nominal / max(weight, 0.01)
        else:
            self.ukf.R = self._R_nominal.copy()
        self.ukf.update(z)

    def pose(self) -> tuple:
        return tuple(self.ukf.x[:3])

    @staticmethod
    def _fx(state, dt, **kwargs):
        x, y, heading, speed, accel, yaw_rate = state
        heading += yaw_rate * dt
        heading = np.arctan2(np.sin(heading), np.cos(heading))
        speed += accel * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
        return np.array([x, y, heading, speed, accel, yaw_rate])

    @staticmethod
    def _hx(state):
        return state[:3]
