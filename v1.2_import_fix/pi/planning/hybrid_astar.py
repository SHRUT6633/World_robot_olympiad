import heapq
import math
import numpy as np


class HybridAStar:
    def __init__(self, step_size=0.1, max_steer=np.radians(30)):
        self.step = step_size
        self.max_steer = max_steer

    def search(self, start, goal, occupancy_grid):
        heap = [(0, 0, start, [start])]
        visited = set()
        while heap:
            f, g, state, path = heapq.heappop(heap)
            key = (round(state[0], 2), round(state[1], 2))
            if key in visited:
                continue
            visited.add(key)
            if np.linalg.norm(np.array(state[:2]) - np.array(goal[:2])) < 0.15:
                return path
            for steer in np.linspace(-self.max_steer, self.max_steer, 7):
                nx = state[0] + self.step * np.cos(state[2] + steer)
                ny = state[1] + self.step * np.sin(state[2] + steer)
                ntheta = state[2] + steer
                ng = g + self.step
                nh = np.linalg.norm(np.array([nx, ny]) - np.array(goal[:2]))
                heapq.heappush(heap, (ng + nh, ng, (nx, ny, ntheta), path + [(nx, ny)]))
        return path
