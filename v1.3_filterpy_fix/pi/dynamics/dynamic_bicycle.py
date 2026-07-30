import numpy as np


class DynamicBicycleModel:
    def __init__(self, mass=2.0, Iz=0.1, lf=0.13, lr=0.13, Cf=50, Cr=50):
        self.m = mass
        self.Iz = Iz
        self.lf = lf
        self.lr = lr
        self.Cf = Cf
        self.Cr = Cr

    def update(self, state, delta, dt):
        x, y, psi, vx, vy, psi_dot = state
        Fyf = -self.Cf * np.arctan((vy + self.lf * psi_dot) / (vx + 1e-6) - delta)
        Fyr = -self.Cr * np.arctan((vy - self.lr * psi_dot) / (vx + 1e-6))
        vx_dot = 0.0
        vy_dot = (Fyf + Fyr) / self.m - vx * psi_dot
        psi_ddot = (self.lf * Fyf - self.lr * Fyr) / self.Iz
        return np.array([
            x + (vx * np.cos(psi) - vy * np.sin(psi)) * dt,
            y + (vx * np.sin(psi) + vy * np.cos(psi)) * dt,
            psi + psi_dot * dt,
            vx + vx_dot * dt,
            vy + vy_dot * dt,
            psi_dot + psi_ddot * dt,
        ])
