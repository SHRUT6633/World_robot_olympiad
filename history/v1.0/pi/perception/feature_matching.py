import cv2
import numpy as np


class FeatureMatcher:
    def __init__(self, nfeatures=500):
        self.orb = cv2.ORB_create(nfeatures=nfeatures)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def extract(self, img):
        kp, des = self.orb.detectAndCompute(img, None)
        return kp, des

    def match(self, des1, des2):
        if des1 is None or des2 is None:
            return []
        matches = self.bf.match(des1, des2)
        return sorted(matches, key=lambda x: x.distance)
