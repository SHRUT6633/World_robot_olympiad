import numpy as np


class DepthEstimator:
    def __init__(self, baseline_m=0.1, focal_px=300):
        self.baseline = baseline_m
        self.focal = focal_px

    def from_disparity(self, disparity):
        mask = disparity > 0
        depth = np.zeros_like(disparity, dtype=float)
        depth[mask] = (self.focal * self.baseline) / disparity[mask]
        return depth
