import cv2
import numpy as np


class CornerDetector:
    def __init__(self, max_corners=50, quality=0.01, min_dist=10):
        self.max_corners = max_corners
        self.quality = quality
        self.min_dist = min_dist

    def detect(self, img):
        if img is None:
            return None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray, self.max_corners, self.quality, self.min_dist)
        if corners is None:
            return np.array([])
        return corners.reshape(-1, 2)
