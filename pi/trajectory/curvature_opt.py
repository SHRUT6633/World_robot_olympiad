import numpy as np


class CurvatureOptimizer:
    # ------------------------------------------------------------------
    # 1) compute_curvature(path) -> 1-D array of curvature values.
    #
    #    Given a path as a sequence of (x, y) points, approximate the
    #    curvature at each point using the formula for a parametrically
    #    defined curve:
    #
    #        κ = |x' y'' - y' x''| / (x'² + y'²)^{3/2}
    #
    #    where primes denote derivatives.  Here:
    #      dx, dy   = first  numerical derivatives (gradient)
    #      ddx, ddy = second numerical derivatives (gradient of gradient)
    #
    #    The +1e-6 in the denominator prevents division by zero when the
    #    path has two consecutive identical points (zero speed).
    #
    #    Return shape:
    #      Same length as the input path.  Curvature is signed only by
    #      absolute value here (we use np.abs).
    #
    #    Typical values:
    #      Straight line   → κ ≈ 0
    #      Sharp turn      → κ > 5
    #      Very sharp turn → κ > 20
    #
    #    What happens if you remove the 1e-6?
    #      A division by zero raises a RuntimeWarning/exception when
    #      the robot is stationary or duplicate waypoints exist.
    # ------------------------------------------------------------------
    @staticmethod
    def compute_curvature(path):
        pts = np.array(path)
        dx = np.gradient(pts[:, 0])
        dy = np.gradient(pts[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = np.abs(ddx * dy - dx * ddy) / (dx**2 + dy**2 + 1e-6)**1.5
        return curvature

    # ------------------------------------------------------------------
    # 2) optimize(path, max_curvature=5.0) -> smoothed path (numpy array)
    #
    #    Iteratively "flatten" path segments whose curvature exceeds a
    #    user-defined threshold (max_curvature).  The result is a path
    #    that the robot can physically follow without tipping or
    #    slipping.
    #
    #    Algorithm:
    #      1) Convert path to a numpy array (N×2).
    #      2) Repeat 10 times:
    #           a) Compute curvature at every point.
    #           b) For each interior point i where curv[i] > max_curvature,
    #              replace pt[i] with the average of its neighbours:
    #                  pt[i] = 0.5 * (pt[i-1] + pt[i+1])
    #              This is a simple "corner cutting" relaxation.
    #      3) Return the modified array.
    #
    #    Why 10 iterations?
    #      Empirically, a handful of iterations is enough to bring most
    #      sharp corners under the threshold.  More iterations would
    #      oversmooth; fewer might leave sharp points.
    #
    #    Important caveat:
    #      - The first and last points are NEVER moved (they are clamped
    #        by the range(1, len-1) loop).  This preserves the start and
    #        goal positions.
    #      - The method modifies the path *in-place* (repeatedly updates
    #        pts[i]), so later curvature calculations see the smoothed
    #        neighbour values.
    #
    #    Connection to the system:
    #      - Receives a path from BezierCurve or WaypointOptimizer.
    #      - Output feeds into JerkMinimizer / AccelerationLimiter to
    #        generate the final motion profile.
    #      - If max_curvature is too high (e.g. 50), sharp corners
    #        remain, potentially causing the robot to skid or the motor
    #        controller to saturate.
    #      - If max_curvature is too low (e.g. 0.5), the path is
    #        aggressively straightened, which may cause the robot to
    #        cut corners and hit obstacles.
    # ------------------------------------------------------------------
    def optimize(self, path, max_curvature=5.0):
        pts = np.array(path)
        for _ in range(10):
            curv = self.compute_curvature(pts)
            for i in range(1, len(pts) - 1):
                if curv[i] > max_curvature:
                    pts[i] = 0.5 * (pts[i - 1] + pts[i + 1])
        return pts
