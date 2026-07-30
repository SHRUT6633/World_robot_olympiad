import numpy as np


class CurvatureOptimizer:
    @staticmethod
    def compute_curvature(path):
        pts = np.array(path)
        dx = np.gradient(pts[:, 0])
        dy = np.gradient(pts[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = np.abs(ddx * dy - dx * ddy) / (dx**2 + dy**2 + 1e-6)**1.5
        return curvature

    def optimize(self, path, max_curvature=5.0):
        pts = np.array(path)
        for _ in range(10):
            curv = self.compute_curvature(pts)
            for i in range(1, len(pts) - 1):
                if curv[i] > max_curvature:
                    pts[i] = 0.5 * (pts[i - 1] + pts[i + 1])
        return pts
