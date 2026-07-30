import cv2
import numpy as np


class StartDetector:
    def __init__(self):
        self._template = None
        self._threshold = 0.7

    def set_template(self, img):
        self._template = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def detect(self, img):
        if self._template is None or img is None:
            return False
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(gray, self._template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, _ = cv2.minMaxLoc(result)
        return max_val >= self._threshold
