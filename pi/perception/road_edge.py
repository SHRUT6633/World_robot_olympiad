# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/road_edge.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Road edge detection via vertical ROI analysis
# =============================================================================

import cv2
import numpy as np


class RoadEdgeDetector:
    # ------------------------------------------------------------------
    # 1) Constructor: defines the vertical Region Of Interest (ROI) as a
    #    fraction of the image height.
    #
    #    roi=(0.4, 0.9)
    #      – The lower 50 % of the image is analysed (from 40 % down to
    #        90 % from the top).  The top 40 % often contains sky /
    #        distant scenery and is irrelevant for near-field road edges.
    #      – Changing roi[0] to, say, 0.6 would move the crop lower and
    #        possibly miss the road entry point; changing roi[1] to 0.99
    #        would include the very front bumper area (more noise).
    # ------------------------------------------------------------------
    def __init__(self, roi=(0.4, 0.9)):
        self.roi = roi

    # ------------------------------------------------------------------
    # 2) detect(img) -> dict or None
    #
    #    Extracts left and right road edges from a single video frame.
    #
    #    Step 1 – Guard clause for None input.
    #
    #    Step 2 – Clip the ROI from the raw frame:
    #             y0 = round(height * 0.4), y1 = round(height * 0.9).
    #             Then work only on that slice (faster, less noise).
    #
    #    Step 3 – Canny edge detection (low=50, high=150).  Every pixel
    #             with gradient magnitude above 150 is a strong edge;
    #             between 50-150 is a weak edge (only kept if connected
    #             to a strong edge).  Raising thresholds reduces
    #             detected edges (good for well-lit roads); lowering them
    #             improves detection in dim light but adds noise.
    #
    #    Step 4 – Left edge extraction:
    #             Split the ROI horizontally at w//2.
    #             For each row (pixel line) in the left half,
    #             np.argmax(edges, axis=1) returns the *first* column
    #             index where a white pixel (edge) occurs, scanning from
    #             left to right.  This works because the left road edge
    #             is typically the first strong vertical edge on the
    #             left side of the image.
    #
    #    Step 5 – Right edge extraction:
    #             np.fliplr() mirrors the right half horizontally so that
    #             argmax again finds the *first* edge, which now
    #             corresponds to the rightmost edge in the original image
    #             (closest to the centre from the right side).
    #             The result is then "un-flipped" conceptually: we treat
    #             the index as distance from the right edge.
    #
    #    Step 6 – Sanitise: any row where argmax returned 0 means no
    #             edge was found (because the pixel column 0 is the
    #             border).  We set those entries to -1 (sentinel value)
    #             so downstream code can ignore missing edges.
    #
    #    Return value:
    #      { "left_edge":  left_array,   # shape (roi_height,)
    #        "right_edge": right_array }  # each entry is column index or -1
    #
    #    Connection to the system:
    #      - These edge profiles feed into the path / trajectory planners
    #        so the robot knows where the drivable corridor is.
    #      - A left_edge of all -1s means "no road edge found" – the
    #        planner may fall back to a different navigation mode.
    # ------------------------------------------------------------------
    def detect(self, img):
        if img is None:
            return None
        h, w = img.shape[:2]
        y0, y1 = int(h * self.roi[0]), int(h * self.roi[1])
        roi = img[y0:y1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        left_edge = np.argmax(edges[:, :w//2], axis=1)
        right_edge = np.argmax(np.fliplr(edges[:, w//2:]), axis=1)
        left_edge[left_edge == 0] = -1
        right_edge[right_edge == 0] = -1
        return {"left_edge": left_edge, "right_edge": right_edge}
