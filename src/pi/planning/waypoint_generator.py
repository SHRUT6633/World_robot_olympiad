# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/planning/waypoint_generator.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Waypoint generation
# =============================================================================

import numpy as np


class WaypointGenerator:
    # WaypointGenerator interpolates a coarse path (e.g. from Hybrid A* or
    # the global planner) into a dense sequence of waypoints at a fixed
    # spacing.  The dense waypoints are then passed to the trajectory
    # smoother and the cubic spline module for fine-grained control.

    def __init__(self, spacing=0.1):
        # spacing -- desired distance (metres) between consecutive waypoints.
        # Smaller values produce more points, which can make the trajectory
        # smoother but increase computation.  0.1 m is reasonable for
        # indoor robot navigation.
        self.spacing = spacing

    def from_path(self, path):
        # path -- list of (x, y) tuples representing a coarse path.
        # Returns a list of (x, y) tuples at approximately spacing intervals.
        waypoints = []

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            # Euclidean distance between the two segment endpoints.
            dist = np.linalg.norm([x2 - x1, y2 - y1])

            # Number of interpolation points (minimum 2 to include both ends).
            n = max(2, int(dist / self.spacing))

            # Linearly interpolate n points along the segment.
            for t in np.linspace(0, 1, n):
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                waypoints.append((x, y))

        return waypoints
