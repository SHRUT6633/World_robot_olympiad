# =============================================================================
# corner_detection.py — Shi-Tomasi corner detector wrapper (OpenCV)
# =============================================================================
# Detects "good features to track" in an image — corners that are suitable
# for optical-flow or feature-based visual odometry.
# =============================================================================

import cv2                                  # OpenCV (computer vision library)
import numpy as np                          # Array / numerical ops


class CornerDetector:
    """
    Wraps cv2.goodFeaturesToTrack (Shi-Tomasi / Kanade–Lucas–Tomasi detector).

    Parameters (tune these to your scene)
    ----------
    max_corners : int
        Maximum number of corners to return.
        Higher → more features but slower; may also pick up noise.
    quality     : float (0–1)
        Minimum acceptable corner quality (fraction of the best eigenvalue).
        Lower → more corners (including weaker ones); higher → only the best.
    min_dist    : int (pixels)
        Minimum Euclidean distance between returned corners.
        Lower → corners can be denser; higher → corners are spread out.

    Connecting to the system
    ------------------------
    - Used by visual_odometry.py and stereo_matching.py to get point
      correspondences between consecutive frames.
    - The output (N×2 array) is fed directly to cv2.calcOpticalFlowPyrLK.
    """

    def __init__(self, max_corners=50, quality=0.01, min_dist=10):
        self.max_corners = max_corners        # Max features to keep
        self.quality = quality                # Min quality level
        self.min_dist = min_dist              # Min inter-corner spacing (px)

    # ------------------------------------------------------------------
    # detect — Run corner detection on a single image
    # ------------------------------------------------------------------
    def detect(self, img):
        """
        Find corners in *img* (BGR format).

        Steps
        -----
        1. Convert to grayscale.
        2. Call goodFeaturesToTrack with the configured parameters.
        3. Reshape from (N, 1, 2) → (N, 2) for convenience.

        Returns
        -------
        np.ndarray of shape (N, 2) or empty array if no corners found.
        None if *img* is None.

        What if you change max_corners/quality/min_dist?
        - max_corners=200  → more corners tracked (better coverage, slower).
        - quality=0.001    → many weak corners included (good for texture-less
          areas, but more outliers).
        - min_dist=5       → corners can cluster (better for fine detail,
          worse for tracking robustness).
        """
        if img is None:                       # Safety guard for missing frames
            return None

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)          # Grayscale
        corners = cv2.goodFeaturesToTrack(                     # Shi-Tomasi
            gray, self.max_corners, self.quality, self.min_dist
        )

        if corners is None:                   # OpenCV returns None when no corners
            return np.array([])               # Return empty array instead

        return corners.reshape(-1, 2)         # Flatten: (N, 1, 2) → (N, 2)
