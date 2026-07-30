# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/fusion/ukf.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Unscented Kalman Filter for state estimation
# =============================================================================

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from ..system.logger import log


class RobotUKF:
    # Unscented Kalman Filter for robot state estimation.
    # State vector (6D): [x (m), y (m), heading (rad), velocity (m/s), acceleration (m/s²), yaw_rate (rad/s)]
    # Measurement vector (6D): same layout (direct observation of full state).
    #
    # The UKF uses sigma points to handle non-linear motion (the _fx function) without
    # requiring Jacobians (unlike the EKF).

    def __init__(self, dt=0.01):
        self.dt = dt
        self.dim_x = 6  # State dimension
        self.dim_z = 6  # Measurement dimension

        # MerweScaledSigmaPoints configures how sigma points are spread.
        # alpha=0.1  -> points close to mean (good for mild non-linearities)
        # beta=2.0   -> optimal for Gaussian distributions
        # kappa=0    -> secondary scaling parameter (default for state estimation)
        points = MerweScaledSigmaPoints(n=self.dim_x, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x, dim_z=self.dim_z, dt=dt, points=points,
            fx=self._fx, hx=self._hx
        )

        # Initial state: all zeros (robot starts at origin, stationary)
        self.ukf.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Initial covariance: 0.1 on diagonal — moderate uncertainty
        self.ukf.P = np.eye(self.dim_x) * 0.1
        # Process noise: 0.001 — assumes fairly accurate motion model
        # Larger Q = more trust in measurements over predictions
        self.ukf.Q = np.eye(self.dim_x) * 0.001
        # Measurement noise: 0.1 — assumes sensors have moderate noise
        # Larger R = more trust in predictions over measurements
        self.ukf.R = np.eye(self.dim_z) * 0.1

    @staticmethod
    def _fx(state, dt):
        # Non-linear state transition function (constant-turn + constant-acceleration model).
        # x += v * cos(heading) * dt
        # y += v * sin(heading) * dt
        # heading += yaw_rate * dt
        # v += a * dt
        # acceleration and yaw_rate persist (random-walk).
        x, y, heading, v, a, yaw_rate = state
        x += v * np.cos(heading) * dt
        y += v * np.sin(heading) * dt
        heading += yaw_rate * dt
        v += a * dt
        return np.array([x, y, heading, v, a, yaw_rate])

    @staticmethod
    def _hx(state):
        # Measurement function: identity — we measure all 6 states directly.
        x, y, heading, v, a, yaw_rate = state
        return np.array([x, y, heading, v, a, yaw_rate])

    def predict(self, dt=None):
        # Advances the state by one time step using the motion model.
        # Uses stored dt unless overridden.
        if dt:
            self.ukf.dt = dt
        self.ukf.predict()

    def update(self, z):
        # Incorporates a measurement vector z (6-element array) to correct the state estimate.
        self.ukf.update(z)

    @property
    def state(self):
        # Returns a copy of the current state estimate [x, y, heading, v, a, yaw_rate].
        return self.ukf.x.copy()

    @property
    def covariance(self):
        # Returns a copy of the state covariance matrix (6x6).
        return self.ukf.P.copy()

    def set_process_noise(self, Q):
        # Override the process noise covariance Q.
        # Useful for adaptive noise tuning at runtime.
        self.ukf.Q = Q

    def set_measurement_noise(self, R):
        # Override the measurement noise covariance R.
        self.ukf.R = R

    def set_state(self, x, P=None):
        # Force the filter state (and optionally covariance) to given values.
        # Used for hard resets (e.g., after a known relocation).
        self.ukf.x = np.array(x)
        if P is not None:
            self.ukf.P = np.array(P)
