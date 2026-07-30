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

    def __init__(self, sobel_thresh=(30, 150)):
        # sobel_thresh -- (lower, upper) bounds for the Sobel magnitude.
        # The lower bound is not directly used here (only the upper for the
        # binary threshold).  In practice:
        #   sobel_thresh[0] = lower hysteresis (reserved for future use).
        #   sobel_thresh[1] = upper threshold: pixels with magnitude above
        #                     this are considered edges (not free).
        # Increasing sobel_thresh[1] makes the detector more permissive
        # (more of the image is labelled free); decreasing it labels more
        # areas as occupied.
        self.sobel_thresh = sobel_thresh

    def detect(self, img):
        # img -- BGR image from the camera (OpenCV format).
        # Returns a grayscale binary image (uint8) where free = 255 and
        # edge/obstacle = 0, or None if img is None.
        if img is None:
            return None

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Compute horizontal and vertical Sobel gradients (5x5 kernel).
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)

        # Magnitude of the gradient vector at each pixel.
        mag = np.sqrt(sobelx**2 + sobely**2)

        # Binary threshold: pixels with magnitude above sobel_thresh[1] are
        # edges -> set to 0 (occupied).  Everything else -> 255 (free).
        # THRESH_BINARY_INV flips the polarity so free space is bright.
        _, free = cv2.threshold(
            mag.astype(np.uint8), self.sobel_thresh[1], 255, cv2.THRESH_BINARY_INV
        )

        return free
