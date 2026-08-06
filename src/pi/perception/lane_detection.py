# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/lane_detection.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Lane line detection via edge/Hough transform
# =============================================================================

import cv2
import numpy as np


class LaneDetector:
    # LaneDetector identifies lane lines in a camera image using Canny edge
    # detection and the probabilistic Hough line transform.  It is a core
    # perception module that tells the robot where the lane boundaries are
    # so the controller can steer to stay centred.  The result feeds into
    # the state machine and Stanley controller.
    #
    # Why Canny + Hough instead of a learned approach?
    #   - The WRO track has well-defined white lane lines on a dark floor.
    #     Edge detection is reliable and interpretable.
    #   - No training data or labelled dataset required.
    #   - Runs at real-time on a Raspberry Pi 4/5 without a GPU.

    def __init__(self, roi_ratio=0.4):
        # roi_ratio — fraction of the image height taken from the bottom.
        # The road / lane markings generally occupy the lower portion of the
        # frame, so we crop everything above this line to reduce noise and
        # processing time.  Default 0.4 means we keep the bottom 40 %.
        # Increasing it widens the search region (more sky / horizon) which
        # can introduce false edges; decreasing it makes the detector blind
        # to markings further ahead.
        self.roi_ratio = roi_ratio

    def detect(self, img):
        # Returns a NumPy array of detected line segments, each with
        # keys "x1", "y1", "x2", "y2" (pixel coordinates).  Returns an empty
        # array when no lines are found, or None if img is None.
        if img is None:
            return None

        h, w = img.shape[:2]

        # Compute the row index where the ROI starts (e.g. bottom 40 %).
        # The top 60 % contains the horizon, walls, ceiling, etc. — nothing
        # useful for lane keeping.
        roi_start = int(h * (1 - self.roi_ratio))

        # Convert the cropped region to grayscale and blur to suppress noise.
        # Gaussian blur (5×5) smooths out floor texture that would otherwise
        # produce spurious Canny edges.
        gray = cv2.cvtColor(img[roi_start:], cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection: thresholds (50, 150) control sensitivity.
        #   - Pixels with gradient magnitude > 150 are strong edges.
        #   - Pixels between 50–150 are weak edges (kept only if connected
        #     to a strong edge).
        # Lowering (50, 150) → more edges (including floor texture noise).
        # Raising    → fewer edges (may miss faint or distant lane lines).
        edges = cv2.Canny(blurred, 50, 150)

        # Probabilistic Hough Line Transform to find finite line segments.
        # Parameters:
        #   rho=1            — 1-pixel distance resolution.
        #   theta=pi/180     — 1-degree angular resolution.
        #   threshold=30     — min votes in accumulator to emit a line.
        #   minLineLength=20 — discard segments shorter than 20 px (noise).
        #   maxLineGap=50    — merge segments separated by ≤ 50 px (useful
        #                      when dashes in a dashed lane line are detected
        #                      as separate segments).
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, 30, minLineLength=20, maxLineGap=50
        )

        lanes = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # y1, y2 are relative to the ROI crop, so add roi_start to
                # recover coordinates in the original (full-frame) image.
                # This way callers (e.g. the controller) can overlay lines
                # on the full image without worrying about the crop.
                lanes.append({
                    "x1": x1, "y1": y1 + roi_start,
                    "x2": x2, "y2": y2 + roi_start,
                })

        return np.array(lanes) if lanes else np.array([])
