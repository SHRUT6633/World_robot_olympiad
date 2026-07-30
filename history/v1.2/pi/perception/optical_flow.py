import cv2
import numpy as np


class OpticalFlow:
    def __init__(self):
        self._prev_gray = None
        self._lk_params = dict(
            winSize=(15, 15), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

    def compute(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self._prev_gray is None:
            self._prev_gray = gray
            return None
        corners = cv2.goodFeaturesToTrack(self._prev_gray, 50, 0.01, 5)
        if corners is None:
            self._prev_gray = gray
            return None
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, corners, None, **self._lk_params
        )
        self._prev_gray = gray
        good = status.ravel() == 1
        if not good.any():
            return None
        flow = next_pts[good] - corners[good]
        return np.mean(flow, axis=0)
