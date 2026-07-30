# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/planning/waypoint_optimizer.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Waypoint optimization
# =============================================================================

import numpy as np


class WaypointOptimizer:
    # ------------------------------------------------------------------
    # 1) Constructor: sets the smoothing strength.
    #
    #    curvature_weight=0.1
    #      – Controls how much each waypoint is pulled toward the
    #        midpoint of its neighbours (a simple Laplacian smooth).
    #      – 0.0 → waypoints are not modified at all (purely follows
    #        the original noisy path).
    #      – 1.0 → waypoints become a straight-line interpolation
    #        between neighbours (oversmoothing, may cut corners).
    #      – 0.1 is a conservative blend: 90 % original, 10 % smoothed.
    # ------------------------------------------------------------------
    def __init__(self, curvature_weight=0.1):
        self.curvature_weight = curvature_weight

    # ------------------------------------------------------------------
    # 2) optimize(waypoints) -> list[(x, y), ...]
    #
    #    Applies a "relaxation" / smoothing pass to a list of waypoints.
    #    This reduces path curvature and makes the trajectory easier
    #    for the low-level controller to follow.
    #
    #    Guard clause:
    #      If there are fewer than 4 waypoints, no smoothing is possible
    #      (there are not enough interior points to relax), so return
    #      them as-is.
    #
    #    Algorithm:
    #      1) Convert the list to a numpy array (N×2).
    #      2) Make a copy `opt` that will be iteratively updated.
    #      3) Run 20 iterations of relaxation:
    #           For each interior point i (skip first and last),
    #             smooth = average of neighbours (opt[i-1] + opt[i+1]) / 2
    #             orig  = original point at this index (from pts, not opt)
    #             opt[i] = (1-w) * orig  +  w * smooth
    #
    #         Why use `opt[i-1]` and `opt[i+1]` (updated values) while
    #         `orig` stays fixed?  This is a *Gauss-Seidel*-like scheme:
    #         using the latest neighbours accelerates convergence, but
    #         anchoring to the original position prevents the entire
    #         path from drifting.
    #
    #      4) Convert back to a list of (x, y) tuples.
    #
    #    Number of iterations (20):
    #      – More iterations = more smoothing, but diminishing returns.
    #      – 20 is a trade-off between smoothness and latency.
    #
    #    Connection to the system:
    #      - Receives waypoints from the global planner (or from
    #        DynamicObstacleAvoidance-adjusted waypoints).
    #      - Output is passed to CurvatureOptimizer and BezierCurve
    #        for fine-grained trajectory generation.
    # ------------------------------------------------------------------
    def optimize(self, waypoints):
        if len(waypoints) < 4:
            return waypoints
        pts = np.array(waypoints)
        opt = pts.copy()
        for _ in range(20):
            for i in range(1, len(pts) - 1):
                smooth = 0.5 * (opt[i - 1] + opt[i + 1])
                orig = pts[i]
                opt[i] = (1 - self.curvature_weight) * orig + self.curvature_weight * smooth
        return [(p[0], p[1]) for p in opt]
