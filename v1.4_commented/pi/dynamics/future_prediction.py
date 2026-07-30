import numpy as np


class FuturePositionPredictor:
    # Predicts the robot's future trajectory using a simple kinematic model
    # (constant velocity + kinematic bicycle steering).
    # This is useful for path planning, obstacle avoidance, and model-predictive
    # control approaches where we need to look ahead.

    def __init__(self, horizon_s=1.0, steps=10):
        # horizon_s: total look-ahead time (seconds).
        #   Longer horizon = see further ahead but less accurate (model error
        #   and unmodeled dynamics accumulate).
        # steps: number of discrete time steps within the horizon.
        #   More steps = finer trajectory resolution but more computation.
        self.horizon = horizon_s
        self.steps = steps

    def predict(self, x, y, heading, v, delta, wheelbase=0.26):
        # x, y: current global position (m).
        # heading: current heading/yaw angle (rad).
        # v: current longitudinal speed (m/s). Assumed constant over the horizon.
        # delta: steering angle (rad). Assumed constant over the horizon.
        # wheelbase: distance between axles (m). Default 0.26 for a 1:10 car.
        # Returns: Nx2 numpy array of (x, y) predicted positions.
        #
        # Kinematic model per step:
        #   x_next = x + v * cos(heading) * dt
        #   y_next = y + v * sin(heading) * dt
        #   heading_next = heading + (v / L) * tan(delta) * dt
        #
        # This assumes no slip and constant speed/steering — a reasonable
        # approximation for low-speed planning. At high speeds, tire slip
        # makes this prediction optimistic (turns will be wider than predicted).
        #
        # Changing horizon_s/steps changes the density and length of the lookahead.
        # Changing wheelbase changes how sharply the robot turns for a given delta:
        #   longer wheelbase = larger predicted turning radius.
        dt = self.horizon / self.steps
        trajectory = []
        cx, cy, ch = x, y, heading
        for _ in range(self.steps):
            cx += v * np.cos(ch) * dt
            cy += v * np.sin(ch) * dt
            ch += (v / wheelbase) * np.tan(delta) * dt
            trajectory.append((cx, cy))
        return np.array(trajectory)
