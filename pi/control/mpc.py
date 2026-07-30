# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/mpc.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Model Predictive Controller for lateral steering control
# =============================================================================

import numpy as np
from ..dynamics.kinematic_model import KinematicModel


class MPCController:
    # Model Predictive Controller for steering (lateral control).
    #
    # At each step, it evaluates multiple candidate steering angles by simulating
    # the robot's kinematic model N steps (horizon) into the future.
    # The candidate with the lowest cost (sum of squared position errors + steering penalty)
    # is selected.
    #
    # This is a brute-force MPC (no optimiser) — it tries a fixed set of discrete
    # steering angles and picks the best. Suitable for low-dimensional problems
    # when the control frequency is modest.
    #
    # horizon (N): number of simulation steps ahead.
    #   Longer horizon = better lookahead but higher computation cost.
    #   Too short = myopic, may miss upcoming turns.
    # dt: simulation time step (seconds). Should match the control loop rate.
    # wheelbase: distance between front and rear axles (meters). Affects turning radius.

    def __init__(self, horizon=10, dt=0.1, wheelbase=0.26):
        self.N = horizon             # Prediction horizon (number of steps)
        self.dt = dt                 # Simulation time step (seconds)
        self.model = KinematicModel(wheelbase)  # Forward simulation model

    def compute(self, x0, y0, heading0, target_path, v):
        # x0, y0, heading0: robot's current pose
        # target_path: list of (x, y) waypoints ahead of the robot (length >= N preferred)
        # v: constant forward speed assumed over the horizon (m/s)
        #
        # Returns the best steering angle (radians) from the candidate set.
        best_steer = 0.0
        best_cost = float("inf")
        # Evaluate 11 discrete steering angles from -0.5 rad (~-28.6°) to +0.5 rad
        for steer in np.linspace(-0.5, 0.5, 11):
            cost = 0.0
            cx, cy, ch = x0, y0, heading0
            # Simulate forward N steps
            for i in range(self.N):
                cx, cy, ch = self.model.update(cx, cy, ch, v, steer, self.dt)
                if i < len(target_path):
                    tx, ty = target_path[i]
                    cost += (cx - tx)**2 + (cy - ty)**2  # Position error cost
                # Steering effort penalty (discourages large angles)
                cost += 0.1 * steer**2
            if cost < best_cost:
                best_cost = cost
                best_steer = steer
        return best_steer
