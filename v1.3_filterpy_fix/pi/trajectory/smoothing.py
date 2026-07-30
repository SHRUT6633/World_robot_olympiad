import numpy as np
from scipy.ndimage import gaussian_filter1d


class TrajectorySmoother:
    @staticmethod
    def gaussian_smooth(path, sigma=2):
        pts = np.array(path)
        if len(pts) < 3:
            return pts
        smoothed = np.zeros_like(pts)
        smoothed[:, 0] = gaussian_filter1d(pts[:, 0], sigma=sigma)
        smoothed[:, 1] = gaussian_filter1d(pts[:, 1], sigma=sigma)
        return smoothed
