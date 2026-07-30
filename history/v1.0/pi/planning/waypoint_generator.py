import numpy as np


class WaypointGenerator:
    def __init__(self, spacing=0.1):
        self.spacing = spacing

    def from_path(self, path):
        waypoints = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dist = np.linalg.norm([x2 - x1, y2 - y1])
            n = max(2, int(dist / self.spacing))
            for t in np.linspace(0, 1, n):
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                waypoints.append((x, y))
        return waypoints
