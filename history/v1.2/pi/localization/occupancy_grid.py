import numpy as np
from ..system.logger import log


class OccupancyGrid:
    def __init__(self, width_m=10, height_m=10, resolution_m=0.05):
        self.width = int(width_m / resolution_m)
        self.height = int(height_m / resolution_m)
        self.resolution = resolution_m
        self.grid = np.full((self.height, self.width), 0.5, dtype=np.float32)
        self.log_odds = np.zeros_like(self.grid)
        self.origin_x = self.width // 2
        self.origin_y = self.height // 2

    def world_to_grid(self, x, y):
        gx = int((x / self.resolution) + self.origin_x)
        gy = int((y / self.resolution) + self.origin_y)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def update_bresenham(self, robot_x, robot_y, obstacle_x, obstacle_y, prob_occ=0.6, prob_free=0.4):
        rx, ry = self.world_to_grid(robot_x, robot_y)
        ox, oy = self.world_to_grid(obstacle_x, obstacle_y)
        l_occ = np.log(prob_occ / (1 - prob_occ))
        l_free = np.log(prob_free / (1 - prob_free))

        dx, dy = abs(ox - rx), abs(oy - ry)
        sx = 1 if ox > rx else -1
        sy = 1 if oy > ry else -1
        err = dx - dy
        x, y = rx, ry

        while True:
            if (x, y) == (ox, oy):
                if 0 <= y < self.height and 0 <= x < self.width:
                    self.log_odds[y, x] += l_occ
                break
            if 0 <= y < self.height and 0 <= x < self.width:
                self.log_odds[y, x] += l_free
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        self.grid = 1 - 1 / (1 + np.exp(self.log_odds))

    def get_free_space(self):
        return self.grid < 0.3

    def get_occupied_space(self):
        return self.grid > 0.7

    def get_unknown_space(self):
        return (self.grid >= 0.3) & (self.grid <= 0.7)
