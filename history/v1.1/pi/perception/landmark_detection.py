import cv2
import numpy as np


class LandmarkDetector:
    def __init__(self):
        self._templates = {}

    def add_template(self, name, img):
        kp, des = cv2.ORB_create().detectAndCompute(img, None)
        self._templates[name] = (kp, des)

    def detect(self, img, threshold=30):
        orb = cv2.ORB_create()
        kp, des = orb.detectAndCompute(img, None)
        if des is None:
            return {}
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        results = {}
        for name, (_, tdes) in self._templates.items():
            if tdes is None:
                continue
            matches = bf.match(tdes, des)
            if len(matches) > threshold:
                results[name] = len(matches)
        return results
