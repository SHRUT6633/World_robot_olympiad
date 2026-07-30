# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/planning/global_planner.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Global path planning
# =============================================================================

import math
from ..system.logger import log


class GlobalPlanner:
    # GlobalPlanner generates a high-level sequence of waypoints that define
    # the robot's path around the track.  For the WRO competition, the track
    # is often a simple rectangle, so plan_rectangle() creates four corner
    # points plus a return to the start.  These waypoints are consumed by
    # the CheckpointManager and the local planner / controller.

    def __init__(self):
        # waypoints -- list of (x, y) tuples in metres.
        self.waypoints = []

    def plan_rectangle(self, width_m, height_m):
        # Generate a closed rectangular path with corners at:
        #   (0,0) -> (width,0) -> (width,height) -> (0,height) -> (0,0)
        # width_m  -- track dimension in the X direction (metres).
        # height_m -- track dimension in the Y direction (metres).
        self.waypoints = [
            (0, 0),
            (width_m, 0),
            (width_m, height_m),
            (0, height_m),
            (0, 0),
        ]
        log.info(f"Rectangle plan: {len(self.waypoints)} waypoints")
        return self.waypoints

    def plan_lap(self, track_width, track_length):
        # Convenience wrapper that delegates to plan_rectangle.
        # track_width   -- width of the track (X dimension).
        # track_length  -- length of the track (Y dimension).
        return self.plan_rectangle(track_width, track_length)

    def get_target(self, index):
        # Return the waypoint at the given index, or None if the index is
        # out of range.  The controller uses this sequentially.
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None
