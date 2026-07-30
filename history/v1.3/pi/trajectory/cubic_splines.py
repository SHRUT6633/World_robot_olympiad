import numpy as np
from scipy.interpolate import CubicSpline


class CubicSplineTrajectory:
    def __init__(self, num_points=50):
        self.num = num_points

    def fit(self, waypoints):
        if len(waypoints) < 3:
            return np.array(waypoints)
        pts = np.array(waypoints)
        x, y = pts[:, 0], pts[:, 1]
        t = np.linspace(0, 1, len(pts))
        cs_x = CubicSpline(t, x, bc_type="natural")
        cs_y = CubicSpline(t, y, bc_type="natural")
        t_dense = np.linspace(0, 1, self.num)
        return np.column_stack((cs_x(t_dense), cs_y(t_dense)))
