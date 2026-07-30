import cv2
import numpy as np


class LaneDetector:
    def __init__(self, roi_ratio=0.4):
        self.roi_ratio = roi_ratio

    def detect(self, img):
        if img is None:
            return None
        h, w = img.shape[:2]
        roi_start = int(h * (1 - self.roi_ratio))
        gray = cv2.cvtColor(img[roi_start:], cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=20, maxLineGap=50)
        lanes = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                lanes.append({
                    "x1": x1, "y1": y1 + roi_start,
                    "x2": x2, "y2": y2 + roi_start,
                })
        return np.array(lanes) if lanes else np.array([])
