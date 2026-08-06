# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/fusion/ekf.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Extended Kalman Filter for state estimation
# =============================================================================

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from ..system.logger import log


class RobotEKF(ExtendedKalmanFilter):
    # Extended Kalman Filter for robot state estimation.
    #
    # State vector (6D): [x (m), y (m), heading (rad), velocity (m/s), acceleration (m/s²), yaw_rate (rad/s)]
    # Measurement vector (6D): same layout (direct observation of all states).
    #
    # Unlike the UKF, the EKF linearises the motion model via a Jacobian matrix (F) at each step.
    # This is computationally cheaper but less accurate for strongly non-linear systems.

    def __init__(self, dt=0.01):
        super().__init__(dim_x=6, dim_z=6)
        self.dt = dt
        # Initial state: all zeros (robot starts at origin, stationary)
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Initial covariance: moderate uncertainty
        # EKF predict: P = F @ P @ F.T + Q  (first-order Taylor expansion)
        self.P = np.eye(6) * 0.1
        # Process noise Q: diagonal 0.001 — assumes smooth motion on WRO track
        # Q too high → filter chases noise; Q too low → filter diverges on curves
        self.Q = np.eye(6) * 0.001
        # Measurement noise R: diagonal 0.1 — sensor fusion input quality
        # Tuned so that GPS/odometry corrections are weighted appropriately
        self.R = np.eye(6) * 0.1
        # State transition Jacobian F (linearisation of _fx at current state)
        # Updated every predict() call via analytic partial derivatives
        self.F = np.eye(6)
        # Measurement Jacobian: identity (we observe all states directly)
        # H = d(hx)/dx |_x  — here hx is identity so H = I
        self.H = np.eye(6)

    def predict(self, dt=None):
        # EKF predict step:
        # 1. Apply the non-linear motion model to propagate the state.
        # 2. Compute the Jacobian F at the current state (first-order Taylor series).
        # 3. Update covariance: P = F @ P @ F.T + Q.
        #
        # Weakness vs UKF: linearisation error grows with non-linearity strength;
        # the UKF's sigma-point propagation is accurate to 3rd order for Gaussian
        # inputs, while the EKF is only 1st-order accurate. For WRO track cornering
        # at moderate speeds, EKF linearisation error is tolerable.
        dt = dt or self.dt
        x, y, heading, v, a, yaw_rate = self.x
        # Jacobian F = df/dx |_x of the motion model:
        #   f0: x_next = x + v*cos(heading)*dt   → df0/dheading = -v*sin(h)*dt
        #   f1: y_next = y + v*sin(heading)*dt   → df1/dheading =  v*cos(h)*dt
        #   f2: heading_next = heading + yaw_rate*dt → df2/dyaw_rate = dt
        #   f3: v_next = v + a*dt               → df3/da = dt
        #   f4: a_next = a, f5: yaw_rate_next = yaw_rate  → diagonal = 1
        # All other partial derivatives are zero (no coupling between unrelated states)
        self.F = np.array([
            [1, 0, -v * np.sin(heading) * dt, np.cos(heading) * dt, 0, 0],
            [0, 1,  v * np.cos(heading) * dt, np.sin(heading) * dt, 0, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, dt, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        # State propagation (non-linear) — same analytic form as UKF _fx
        self.x[0] += v * np.cos(heading) * dt
        self.x[1] += v * np.sin(heading) * dt
        self.x[2] += yaw_rate * dt
        self.x[3] += a * dt
        # Covariance propagation (linearised via Jacobian)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        # EKF update (correction) step — Joseph's form covariance update:
        #   Innovation:   y = z - H*x              (measurement residual)
        #   Innov. cov:   S = H*P*H^T + R          (uncertainty in measurement space)
        #   Kalman gain:  K = P*H^T * S^{-1}       (optimal blending weight)
        #   State:        x = x + K*y               (corrected estimate)
        #   Covariance:   P = (I - K*H)*P           (reduced uncertainty after obs.)
        #
        # The Kalman gain K balances prediction vs. measurement:
        #   - R large → K small → measurement discounted (trust model)
        #   - Q large → P large → K large → measurement weighted more
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
