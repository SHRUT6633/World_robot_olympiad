import numpy as np
from scipy.ndimage import gaussian_filter1d


class TrajectorySmoother:
    # TrajectorySmoother applies a Gaussian filter to the X and Y coordinates
    # of a path independently, producing a smoother version that is easier
    # for the controller to follow.  The standard deviation (sigma) controls
    # the amount of smoothing.

    @staticmethod
    def gaussian_smooth(path, sigma=2):
        # path  -- list or array of (x, y) points.
        # sigma -- standard deviation of the Gaussian kernel.  Higher values
        #   produce more aggressively smoothed paths (but may cut corners).
        #   Lower values preserve the original shape better.
        # Returns a NumPy array of the same shape as the input.

        pts = np.array(path)

        # Need at least 3 points for meaningful smoothing.
        if len(pts) < 3:
            return pts

        smoothed = np.zeros_like(pts)
        # Apply a 1D Gaussian filter independently to the X and Y sequences.
        smoothed[:, 0] = gaussian_filter1d(pts[:, 0], sigma=sigma)
        smoothed[:, 1] = gaussian_filter1d(pts[:, 1], sigma=sigma)

        return smoothed
