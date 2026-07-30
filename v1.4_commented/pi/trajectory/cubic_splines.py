import numpy as np
from scipy.interpolate import CubicSpline


class CubicSplineTrajectory:
    # CubicSplineTrajectory takes a set of waypoints and fits a smooth,
    # continuous cubic spline through them.  The resulting dense set of
    # points creates a smooth path that the robot can follow without
    # sharp corners.  The spline uses "natural" boundary conditions
    # (second derivative = 0 at the ends).

    def __init__(self, num_points=50):
        # num_points -- number of evenly-spaced points to sample from the
        # fitted spline.  More points give finer resolution but increase
        # the size of the output array.
        self.num = num_points

    def fit(self, waypoints):
        # waypoints -- list of (x, y) tuples (at least 3 for a meaningful spline).
        # Returns a NumPy array of shape (num_points, 2) with the smoothed path.
        # If fewer than 3 waypoints are supplied, returns the original array
        # unchanged (spline requires >= 3 points).

        if len(waypoints) < 3:
            return np.array(waypoints)

        pts = np.array(waypoints)
        x, y = pts[:, 0], pts[:, 1]

        # Parameter t runs from 0 to 1, one value per input waypoint.
        t = np.linspace(0, 1, len(pts))

        # Fit a cubic spline for X(t) and Y(t) independently.
        # bc_type="natural" means zero second derivative at boundaries,
        # which produces a smooth curve that doesn't overshoot at the ends.
        cs_x = CubicSpline(t, x, bc_type="natural")
        cs_y = CubicSpline(t, y, bc_type="natural")

        # Sample the spline at num_points evenly-spaced parameter values.
        t_dense = np.linspace(0, 1, self.num)

        return np.column_stack((cs_x(t_dense), cs_y(t_dense)))
