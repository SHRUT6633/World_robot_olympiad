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
        self.P = np.eye(6) * 0.1
        # Process noise: trust the motion model fairly well
        self.Q = np.eye(6) * 0.001
        # Measurement noise: default sensor uncertainty
        self.R = np.eye(6) * 0.1
        # State transition Jacobian (will be updated in predict())
        self.F = np.eye(6)
        # Measurement Jacobian: identity (we observe all states directly)
        self.H = np.eye(6)

    def predict(self, dt=None):
        # EKF predict step:
        # 1. Apply the non-linear motion model to propagate the state.
        # 2. Compute the Jacobian F at the current state.
        # 3. Update covariance: P = F @ P @ F.T + Q.
        dt = dt or self.dt
        x, y, heading, v, a, yaw_rate = self.x
        # Jacobian of the motion model:
        #   x_next = x + v*cos(heading)*dt
        #   y_next = y + v*sin(heading)*dt
        #   heading_next = heading + yaw_rate*dt
        #   v_next = v + a*dt
        #   a_next = a
        #   yaw_rate_next = yaw_rate
        self.F = np.array([
            [1, 0, -v * np.sin(heading) * dt, np.cos(heading) * dt, 0, 0],
            [0, 1,  v * np.cos(heading) * dt, np.sin(heading) * dt, 0, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, dt, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        # State propagation (non-linear)
        self.x[0] += v * np.cos(heading) * dt
        self.x[1] += v * np.sin(heading) * dt
        self.x[2] += yaw_rate * dt
        self.x[3] += a * dt
        # Covariance propagation (linearised)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        # EKF update step:
        #   y = z - H*x  (innovation)
        #   S = H*P*H^T + R
        #   K = P*H^T * S⁻¹  (Kalman gain)
        #   x = x + K*y
        #   P = (I - K*H)*P
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
