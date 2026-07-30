import numpy as np


class SafeCorridorGenerator:
    def __init__(self, corridor_width=0.3):
        self.width = corridor_width

    def generate(self, waypoints, occupancy_grid):
        corridors = []
        for wx, wy in waypoints:
            gx, gy = occupancy_grid.world_to_grid(wx, wy)
            half = int(self.width / occupancy_grid.resolution / 2)
            x0, x1 = max(0, gx - half), min(occupancy_grid.width - 1, gx + half)
            y0, y1 = max(0, gy - half), min(occupancy_grid.height - 1, gy + half)
            free = not occupancy_grid.grid[y0:y1, x0:x1].max() > 0.7
            corridors.append({"x": wx, "y": wy, "safe": free})
        return corridors
