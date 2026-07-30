import cv2
import numpy as np


class LaneDetector:
    # LaneDetector identifies lane lines in a camera image using edge detection
    # and the Hough line transform. It is a core perception module that tells the
    # robot where the lane boundaries are so the controller can steer to stay
    # centred. The result feeds into the state machine and Stanley controller.

    def __init__(self, roi_ratio=0.4):
        # roi_ratio -- fraction of the image height taken from the bottom.
        # The road / lane markings generally occupy the lower portion of the
        # frame, so we crop everything above this line to reduce noise and
        # processing time.  Default 0.4 means we keep the bottom 40 %.
        # Increasing it widens the search region (more sky / horizon) which
        # can introduce false edges; decreasing it makes the detector blind
        # to markings further ahead.
        self.roi_ratio = roi_ratio

    def detect(self, img):
        # Returns a NumPy array of detected line segments, each with
        # keys x1, y1, x2, y2 (pixel coordinates).  Returns an empty
        # array when no lines are found, or None if img is None.
        if img is None:
            return None

        h, w = img.shape[:2]

        # Compute the row index where the ROI starts (e.g. 60 % from the top).
        roi_start = int(h * (1 - self.roi_ratio))

        # Convert the cropped region to grayscale and blur to suppress noise.
        gray = cv2.cvtColor(img[roi_start:], cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection: thresholds (50, 150) control sensitivity.
        # Lower values detect more edges (including noise); higher values
        # may miss faint lane markings.
        edges = cv2.Canny(blurred, 50, 150)

        # Probabilistic Hough Line Transform to find line segments in edge
        # space.  Parameters:
        #   rho=1         -- distance resolution in pixels.
        #   theta=pi/180  -- angular resolution in radians.
        #   threshold=30  -- minimum votes to consider a line.
        #   minLineLength=20 -- smallest segment length (pixels) to keep.
        #   maxLineGap=50 -- max gap (pixels) between segments to merge them.
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, 30, minLineLength=20, maxLineGap=50
        )

        lanes = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # y1, y2 are relative to the ROI, so add roi_start to recover
                # coordinates in the original (full-frame) image.
                lanes.append({
                    "x1": x1, "y1": y1 + roi_start,
                    "x2": x2, "y2": y2 + roi_start,
                })

        return np.array(lanes) if lanes else np.array([])
