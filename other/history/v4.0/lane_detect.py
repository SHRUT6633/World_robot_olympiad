import cv2
import numpy as np
from collections import deque


class LaneDetect:
    def __init__(self, frame_width=640, frame_height=480, window_size=5):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.window_size = window_size
        self.left_slopes = deque(maxlen=window_size)
        self.right_slopes = deque(maxlen=window_size)
        self.left_intercepts = deque(maxlen=window_size)
        self.right_intercepts = deque(maxlen=window_size)

    def process(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.0)
        edges = cv2.Canny(blurred, 50, 150)

        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=50,
            minLineLength=40, maxLineGap=10
        )

        if lines is None:
            return self._average_lanes()

        left_points = []
        right_points = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if slope < -0.2:
                left_points.extend([(x1, y1), (x2, y2)])
            elif slope > 0.2:
                right_points.extend([(x1, y1), (x2, y2)])

        left_lane = self._fit_line(left_points)
        right_lane = self._fit_line(right_points)

        if left_lane and right_lane:
            left_slope, left_intercept = left_lane
            right_slope, right_intercept = right_lane

            if self._slope_in_range(left_slope, "left") and self._slope_in_range(right_slope, "right"):
                self.left_slopes.append(left_slope)
                self.right_slopes.append(right_slope)
                self.left_intercepts.append(left_intercept)
                self.right_intercepts.append(right_intercept)

        return self._average_lanes()

    def _fit_line(self, points):
        if len(points) < 2:
            return None
        xs = np.array([p[0] for p in points], dtype=np.float64)
        ys = np.array([p[1] for p in points], dtype=np.float64)
        A = np.vstack([xs, np.ones_like(xs)]).T
        slope, intercept = np.linalg.lstsq(A, ys, rcond=None)[0]
        return slope, intercept

    def _slope_in_range(self, slope, side):
        if side == "left":
            return -2.0 < slope < -0.2
        elif side == "right":
            return 0.2 < slope < 2.0
        return False

    def _average_lanes(self):
        if len(self.left_slopes) == 0 or len(self.right_slopes) == 0:
            return None
        left_slope = np.mean(self.left_slopes)
        right_slope = np.mean(self.right_slopes)
        left_intercept = np.mean(self.left_intercepts)
        right_intercept = np.mean(self.right_intercepts)
        return {
            "left": (left_slope, left_intercept),
            "right": (right_slope, right_intercept)
        }
