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
        # The UKF propagates 2*n+1 = 13 sigma points through _fx, then
        # reconstructs the Gaussian from weighted mean/covariance of the
        # transformed points. This avoids Jacobian linearisation errors
        # inherent in the EKF, at O(n^3) vs O(n^2) cost.
        points = MerweScaledSigmaPoints(n=self.dim_x, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x, dim_z=self.dim_z, dt=dt, points=points,
            fx=self._fx, hx=self._hx
        )

        # Initial state: all zeros (robot starts at origin, stationary)
        self.ukf.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Initial covariance: 0.1 on diagonal — moderate uncertainty
        # UKF predict: P = sum(W_c * (sigma_i - x)(sigma_i - x)^T) + Q
        self.ukf.P = np.eye(self.dim_x) * 0.1
        # Process noise Q: 0.001 = small diffusion per step
        # Larger Q inflates sigma-point spread → filter trusts measurements more
        # Chosen empirically: WRO track is smooth (low Q) but not perfect
        self.ukf.Q = np.eye(self.dim_x) * 0.001
        # Measurement noise R: 0.1 = sensor uncertainty budget
        # Larger R shrinks Kalman gain → filter trusts predictions more
        # Trade-off: R too low = jittery, R too high = sluggish response
        self.ukf.R = np.eye(self.dim_z) * 0.1

    @staticmethod
    def _fx(state, dt):
        # Non-linear state transition function (constant-turn + constant-acceleration model).
        # UKF propagates each sigma point through this analytic function;
        # no Jacobian needed — the unscented transform handles the
        # non-linear distribution analytically via weighted sigma points.
        #
        # Motion model equations (discrete-time kinematics):
        #   x_{k+1} = x_k + v_k * cos(heading_k) * dt
        #   y_{k+1} = y_k + v_k * sin(heading_k) * dt
        #   heading_{k+1} = heading_k + yaw_rate_k * dt
        #   v_{k+1} = v_k + a_k * dt
        #   a_{k+1} = a_k,  yaw_rate_{k+1} = yaw_rate_k  (random-walk persistence)
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
        # UKF predict step:
        #   1. Generate sigma points from current x, P (unscented transform)
        #   2. Propagate each point through _fx (non-linear motion)
        #   3. Recombine weighted sigma points into predicted x_pred, P_pred
        #      x_pred = sum(W_m * sigma_i)
        #      P_pred = sum(W_c * (sigma_i - x_pred)(sigma_i - x_pred)^T) + Q
        # Uses stored dt unless overridden.
        if dt:
            self.ukf.dt = dt
        self.ukf.predict()

    def update(self, z):
        # UKF update step:
        #   1. Regenerate sigma points from predicted x_pred, P_pred
        #   2. Propagate through _hx (identity) → predicted measurement z_pred
        #   3. Compute innovation: y = z - z_pred
        #   4. Cross-covariance P_xz → Kalman gain K = P_xz * S^{-1}
        #   5. x = x_pred + K * y,  P = P_pred - K * S * K^T
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
