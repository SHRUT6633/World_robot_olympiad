import cv2
import numpy as np


class ObjectDetector:
    def __init__(self):
        self._bboxes = []

    def detect_contours(self, img, min_area=500):
        if img is None:
            return []
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                objects.append({"bbox": (x, y, w, h), "area": area, "cx": x + w//2, "cy": y + h//2})
        return objects
