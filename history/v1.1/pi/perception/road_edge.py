import cv2
import numpy as np


class RoadEdgeDetector:
    def __init__(self, roi=(0.4, 0.9)):
        self.roi = roi

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
