import numpy as np
from ..dynamics.kinematic_model import KinematicModel


class MPCController:
    def __init__(self, horizon=10, dt=0.1, wheelbase=0.26):
        self.N = horizon
        self.dt = dt
        self.model = KinematicModel(wheelbase)

    def compute(self, x0, y0, heading0, target_path, v):
        best_steer = 0.0
        best_cost = float("inf")
        for steer in np.linspace(-0.5, 0.5, 11):
            cost = 0.0
            cx, cy, ch = x0, y0, heading0
            for i in range(self.N):
                cx, cy, ch = self.model.update(cx, cy, ch, v, steer, self.dt)
                if i < len(target_path):
                    tx, ty = target_path[i]
                    cost += (cx - tx)**2 + (cy - ty)**2
                cost += 0.1 * steer**2
            if cost < best_cost:
                best_cost = cost
                best_steer = steer
        return best_steer
