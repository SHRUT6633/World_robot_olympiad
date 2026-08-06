# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/localization/occupancy_grid.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: 2-D occupancy grid with log-odds Bayesian mapping
# =============================================================================

import numpy as np
from ..system.logger import log


class OccupancyGrid:
    # ──────────────────────────────────────────────────────────────────
    # 2-D occupancy grid that stores a *probability* of each cell
    # being occupied.  Uses log-odds representation for efficient
    # Bayesian updates and Bresenham ray-tracing to mark free /
    # occupied cells from sensor readings.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self, width_m=10, height_m=10, resolution_m=0.05):
        # World dimensions in metres.
        # Convert to cell counts.
        self.width = int(width_m / resolution_m)    # number of columns
        self.height = int(height_m / resolution_m)  # number of rows
        self.resolution = resolution_m              # metres per cell

        # Grid of occupancy probabilities (0 = free, 1 = occupied).
        # Initialised to 0.5 (completely unknown).
        self.grid = np.full((self.height, self.width), 0.5, dtype=np.float32)

        # Log-odds representation; zero corresponds to P=0.5.
        self.log_odds = np.zeros_like(self.grid)

        # Origin (robot start) placed at the centre of the grid.
        self.origin_x = self.width // 2
        self.origin_y = self.height // 2

    def world_to_grid(self, x, y):
        # Convert world coordinates (metres) to grid indices.
        # Rounding via int() truncates toward zero.
        gx = int((x / self.resolution) + self.origin_x)
        gy = int((y / self.resolution) + self.origin_y)
        return gx, gy

    def grid_to_world(self, gx, gy):
        # Inverse of world_to_grid – grid indices → world metres.
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def update_bresenham(self, robot_x, robot_y,
                         obstacle_x, obstacle_y,
                         prob_occ=0.6, prob_free=0.4):
        # ── Bresenham-based ray-trace update ──────────────────────
        # robot_x, robot_y   – sensor origin (world frame).
        # obstacle_x,obstacle_y – detected obstacle (world frame).
        #
        # All cells on the ray from robot→obstacle are marked *free*
        # (except the endpoint which is *occupied*).
        #
        # prob_occ / prob_free are the observation probabilities
        # used to derive log-odds increments.

        rx, ry = self.world_to_grid(robot_x, robot_y)
        ox, oy = self.world_to_grid(obstacle_x, obstacle_y)

        # Convert probabilities to log-odds.
        # l_occ = log( p/(1-p) )  –  added at the endpoint.
        # l_free = log( p/(1-p) ) –  added along the ray.
        l_occ = np.log(prob_occ / (1 - prob_occ))
        l_free = np.log(prob_free / (1 - prob_free))

        # ── Bresenham setup ──────────────────────────────────────
        dx, dy = abs(ox - rx), abs(oy - ry)
        sx = 1 if ox > rx else -1
        sy = 1 if oy > ry else -1
        err = dx - dy
        x, y = rx, ry

        # Walk the ray.
        while True:
            if (x, y) == (ox, oy):          # endpoint → occupied
                if 0 <= y < self.height and 0 <= x < self.width:
                    self.log_odds[y, x] += l_occ
                break

            if 0 <= y < self.height and 0 <= x < self.width:
                self.log_odds[y, x] += l_free   # free cell

            # Bresenham step.
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        # Convert updated log-odds back to probability.
        # P = 1 - 1/(1 + exp(L)) ,  where L = log-odds.
        self.grid = 1 - 1 / (1 + np.exp(self.log_odds))

    # ── Convenience accessors for path-planning ──────────────────

    def get_free_space(self):
        # Boolean mask of cells believed to be free (P < 30 %).
        return self.grid < 0.3

    def get_occupied_space(self):
        # Boolean mask of cells believed to be occupied (P > 70 %).
        return self.grid > 0.7

    def get_unknown_space(self):
        # Cells that are neither confidently free nor occupied.
        return (self.grid >= 0.3) & (self.grid <= 0.7)

# ── What happens if you change key values? ─────────────────────────
# * resolution_m  ↓ (smaller cells) → finer map, more memory / CPU.
#   ↑ (larger cells) → coarser map, faster but less accurate.
# * width_m / height_m – determines total coverage area.
# * prob_occ ↑ → stronger belief in obstacles (faster to P>0.7).
# * prob_free ↑ → stronger belief in free space (faster to P<0.3).
# * The Bresenham loop currently has no bounds check on x,y after
#   stepping – if the ray exits the grid it will keep running.
#   A grid-bounds check inside the loop would prevent infinite loops.
# ────────────────────────────────────────────────────────────────────
