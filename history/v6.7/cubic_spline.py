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

        dists = np.cumsum(np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1)))
        dists = np.insert(dists, 0, 0)
        t = dists / dists[-1]

        start_dx = x[1] - x[0]
        start_dy = y[1] - y[0]
        end_dx = x[-1] - x[-2]
        end_dy = y[-1] - y[-2]

        cs_x = CubicSpline(t, x, bc_type=((1, start_dx), (1, end_dx)))
        cs_y = CubicSpline(t, y, bc_type=((1, start_dy), (1, end_dy)))

        t_dense = np.linspace(0, 1, self.num)
        return np.column_stack((cs_x(t_dense), cs_y(t_dense)))
