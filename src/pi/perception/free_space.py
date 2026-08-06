# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/free_space.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Free drivable space detection
# =============================================================================

import cv2
import numpy as np


class FreeSpaceDetector:
    # FreeSpaceDetector produces a binary image where white pixels represent
    # "free" (low-texture / drivable) area and black pixels represent edges
    # or obstacles.  It uses the Sobel gradient magnitude to find high-frequency
    # regions (edges, texture) and inverts the threshold.  The result is used
    # by the planning modules to know where the robot can safely drive.
    #
    # Algorithm:
    #   1. Convert to grayscale.
    #   2. Compute horizontal and vertical Sobel gradients (5×5 kernel).
    #   3. Magnitude = sqrt(Gx² + Gy²) — captures all strong edges.
    #   4. Threshold: pixels above the threshold are "edges/obstacles" (black),
    #      everything else is "free space" (white).
    #
    # Why Sobel instead of Canny?
    #   - Canny applies non-maximum suppression and hysteresis, which produces
    #     thin, clean edges — great for lane lines, but not for free-space
    #     estimation where we want a "cost map" of textured/occupied regions.
    #   - Sobel magnitude gives a continuous gradient response; thresholding
    #     it produces a coarser but more complete obstacle mask.

    def __init__(self, sobel_thresh=(30, 150)):
        # sobel_thresh — (lower, upper) bounds for the Sobel magnitude.
        #   sobel_thresh[0]: lower bound (reserved for future hysteresis).
        #   sobel_thresh[1]: upper threshold — pixels with magnitude above
        #                    this are classified as occupied.
        # Increasing sobel_thresh[1] makes the detector more permissive
        # (more of the image is labelled free); decreasing it labels more
        # areas as occupied (may cause false obstacle detections on plain
        # floor with slight texture).
        self.sobel_thresh = sobel_thresh

    def detect(self, img):
        # img — BGR image from the camera (OpenCV format).
        # Returns a grayscale binary image (uint8) where free = 255 and
        # edge/obstacle = 0, or None if img is None.
        if img is None:
            return None

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Compute horizontal and vertical Sobel gradients (5×5 kernel).
        # The kernel size controls the scale of edges detected: 5×5 catches
        # medium-width edges (floor joints, wall bases, pillar edges).  A
        # 3×3 kernel would be noisier; 7×7 would be smoother but might miss
        # thin obstacles.
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)

        # Magnitude of the gradient vector at each pixel.
        # We use float64 during computation to avoid overflow, then cast
        # back to uint8 for the threshold.
        mag = np.sqrt(sobelx**2 + sobely**2)

        # Binary threshold: pixels with magnitude above sobel_thresh[1] are
        # considered edges/obstacles → set to 0 (occupied).  Everything else
        # → 255 (free).  THRESH_BINARY_INV inverts the polarity so that free
        # space is bright (255) and obstacles are dark (0), which matches the
        # convention expected by the cost-map / path-planner modules.
        _, free = cv2.threshold(
            mag.astype(np.uint8), self.sobel_thresh[1], 255, cv2.THRESH_BINARY_INV
        )

        return free
