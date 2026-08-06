# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/stanley.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Stanley lateral steering controller for path tracking
# =============================================================================

import numpy as np


class StanleyController:
    # Stanley controller for lateral (steering) control of a wheeled robot.
    # Based on the Stanley method from the DARPA Grand Challenge (Stanford).
    #
    # Steering angle = heading_error + arctan(k * crosstrack_error / (k_soft + speed))
    #
    # - heading_error: difference between target heading and robot heading (rad)
    # - crosstrack_error: lateral distance from robot to the target path (m)
    # - k: gain that determines how aggressively the robot corrects crosstrack error
    # - k_soft: softening constant that prevents division by zero at low speed
    #
    # Increasing k -> more aggressive crosstrack correction (sharper turns, may oscillate).
    # Decreasing k -> smoother but slower convergence to the path.
    # Higher k_soft reduces steering at low speeds (safer, less jerky).
    # max_steering: mechanical steering limit (default ±30 degrees).

    def __init__(self, k=0.5, k_soft=1.0, max_steering=np.radians(30)):
        self.k = k
        self.k_soft = k_soft
        self.max_steering = max_steering

    def compute(self, robot_x, robot_y, robot_heading, target_x, target_y, target_heading, v):
        # Compute error vector from robot to target point in global coordinates
        dx = target_x - robot_x
        dy = target_y - robot_y
        # Cross-track error: rotate (dx, dy) into robot body frame by dot with lateral axis
        # Robot lateral axis = (-sin(ψ), cos(ψ)). Positive = target is left of robot.
        crosstrack = -np.sin(robot_heading) * dx + np.cos(robot_heading) * dy
        # Heading error: difference between desired path heading and robot heading
        heading_error = target_heading - robot_heading
        # arctan2(sin, cos) normalises heading_error to the range [-pi, pi]
        # Prevents 360° wrapping from causing full-lock steering in the wrong direction
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Stanley control law:
        # steering = heading_error + arctan(k * crosstrack / (k_soft + v))
        # The arctan saturates the crosstrack correction between -pi/2 and +pi/2.
        # Stanley law: heading correction + arctan-saturated cross-track correction
        # arctan2 maps crosstrack * k / (k_soft + v) to (-π/2, +π/2), preventing unbounded steering
        # k_soft = 1.0 ensures finite steering even at v=0 (division safety)
        steer = heading_error + np.arctan2(self.k * crosstrack, self.k_soft + v)
        return np.clip(steer, -self.max_steering, self.max_steering)
