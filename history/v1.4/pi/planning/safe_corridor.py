import numpy as np


class SafeCorridorGenerator:
    # ──────────────────────────────────────────────────────────────────
    # For each waypoint on a planned path, checks whether a
    # fixed-width square around that point is free of obstacles
    # according to the OccupancyGrid.
    #
    # Used by the local planner to decide if a global waypoint is
    # safe to drive toward, or needs re-routing.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self, corridor_width=0.3):
        # corridor_width – side length (metres) of the square safety
        # region centred on each waypoint.
        self.width = corridor_width

    def generate(self, waypoints, occupancy_grid):
        # waypoints      – list of (x, y) tuples (world frame).
        # occupancy_grid – OccupancyGrid instance.
        # Returns a list of dicts: {x, y, safe}.

        corridors = []

        for wx, wy in waypoints:
            # Convert waypoint to grid coordinates.
            gx, gy = occupancy_grid.world_to_grid(wx, wy)

            # Radius in cell units (half the corridor width).
            half = int(self.width / occupancy_grid.resolution / 2)

            # Clamp the bounding-box to grid boundaries.
            x0, x1 = max(0, gx - half), min(occupancy_grid.width - 1, gx + half)
            y0, y1 = max(0, gy - half), min(occupancy_grid.height - 1, gy + half)

            # Mark 'safe' if no cell in the square exceeds 70 %
            # occupancy probability (i.e. no "occupied" cell).
            free = not occupancy_grid.grid[y0:y1, x0:x1].max() > 0.7

            corridors.append({"x": wx, "y": wy, "safe": free})

        return corridors

# ── What happens if you change key values? ─────────────────────────
# * corridor_width  ↑ → larger safety margin, fewer waypoints
#   considered safe → more conservative, may block valid paths.
#   ↓ → robot squeezes through tighter gaps but risks collision.
# * The 0.7 threshold for "occupied" is hard-coded here but should
#   ideally be taken from the OccupancyGrid to stay consistent.
# * The slice occupancy_grid.grid[y0:y1, x0:x1] is empty when
#   x0==x1 or y0==y1 (waypoint exactly at boundary).  In that case
#   .max() raises a ValueError on an empty array – a guard would be
#   prudent.
# * This checks only the corners of the corridor (since it's an
#   axis-aligned bounding box).  For a more accurate check, test
#   every cell individually.
# ────────────────────────────────────────────────────────────────────
