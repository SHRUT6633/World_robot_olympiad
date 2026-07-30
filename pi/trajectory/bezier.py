# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/trajectory/bezier.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Bézier curve trajectory generation
# =============================================================================

import numpy as np


class BezierCurve:
    # ------------------------------------------------------------------
    # 1) quadratic(p0, p1, p2, num=20) -> N×2 array
    #
    #    Generates a quadratic Bézier curve from three control points:
    #
    #        B(t) = (1-t)² P0  +  2(1-t)t P1  +  t² P2
    #
    #    where t ∈ [0, 1].
    #
    #    Parameters:
    #      p0, p1, p2 : (x, y) tuples – control points.
    #      P0 = start, P2 = end, P1 = anchor that pulls the curve.
    #      num        : int – number of points to sample (default 20).
    #                  More points = smoother visualisation but more
    #                  computation for downstream curvature checks.
    #
    #    Return:
    #      N×2 numpy array where each row is a (x, y) point along the
    #      curve.  The first row ≅ P0, the last row ≅ P2.
    #
    #    Use case:
    #      Simple curved paths when only three waypoints are available.
    #      The middle point P1 acts as a "handle" – moving P1 farther
    #      from the P0–P2 line makes the curve more pronounced.
    #
    #    What if num=2?
    #      Only the start and end points are returned (a straight line).
    # ------------------------------------------------------------------
    @staticmethod
    def quadratic(p0, p1, p2, num=20):
        t = np.linspace(0, 1, num)
        x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
        y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
        return np.column_stack((x, y))

    # ------------------------------------------------------------------
    # 2) cubic(p0, p1, p2, p3, num=30) -> N×2 array
    #
    #    Generates a cubic Bézier curve from four control points:
    #
    #        B(t) = (1-t)³ P0  +  3(1-t)² t P1
    #             +  3(1-t) t² P2  +  t³ P3
    #
    #    Parameters:
    #      p0, p1, p2, p3 : (x, y) tuples.
    #      P0 = start, P3 = end, P1 / P2 = two anchor handles.
    #      num             : int (default 30) – sample count.
    #
    #    Return:
    #      N×2 numpy array.
    #
    #    Why cubic over quadratic?
    #      Two handles allow an S-shaped curve (inflection point), which
    #      is impossible with a single-handle quadratic.  This makes
    #      cubic more suitable for connecting waypoints that require a
    #      lane-change or obstacle-smoothing manoeuvre.
    #
    #    Connection to the system:
    #      - After WaypointOptimizer and CurvatureOptimizer have prepared
    #        a cleaned set of waypoints, this class converts the discrete
    #        waypoints into a continuous, differentiable path.
    #      - The output array can be fed to a pure-pursuit controller or
    #        used to compute curvature via CurvatureOptimizer.
    # ------------------------------------------------------------------
    @staticmethod
    def cubic(p0, p1, p2, p3, num=30):
        t = np.linspace(0, 1, num)
        x = (1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + 3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
        y = (1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + 3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
        return np.column_stack((x, y))
