import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

ALPHA = 0.1
WINDOW = 50


class AdaptiveUKF:
    def __init__(self):
        points = MerweScaledSigmaPoints(6, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=6, dim_z=3, dt=0.02, fx=self._fx, hx=self._hx, points=points
        )
        self.ukf.x = np.zeros(6)
        self.ukf.P = np.eye(6) * 0.1
        self.ukf.Q = np.eye(6) * 1e-3
        self.ukf.R = np.eye(3) * 1e-1
        self._q_nominal = self.ukf.Q.copy()
        self._ema_innov = np.zeros(3)
        self._ema_cov = np.eye(3) * 1e-3
        self._count = 0

    def predict(self, dt: float) -> None:
        self.ukf.dt = dt
        self.ukf.predict()

    def correct(self, z: np.ndarray) -> None:
        self.ukf.update(z)
        innov = z - self._hx(self.ukf.x)
        self._ema_innov = ALPHA * innov + (1 - ALPHA) * self._ema_innov
        self._ema_cov = (ALPHA * np.outer(innov, innov)
                         + (1 - ALPHA) * self._ema_cov)
        self._count += 1
        if self._count >= WINDOW:
            innov_var = np.trace(self._ema_cov)
            nominal_var = np.trace(self._q_nominal[:3, :3])
            scale = max(0.1, min(10.0, innov_var / nominal_var))
            self.ukf.Q = self._q_nominal * scale

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
