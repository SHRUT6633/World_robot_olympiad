import cv2
import numpy as np


class VisualOdometry:
    def __init__(self, width=320, height=240):
        self.width = width
        self.height = height
        self.prev_gray = None
        self.prev_pts = None
        self.fast = cv2.FastFeatureDetector_create(threshold=20)
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self.position = np.array([0.0, 0.0])
        self.heading = 0.0

    def process(self, frame):
        small = cv2.resize(frame, (self.width, self.height))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        result = {"moved": False, "dx": 0.0, "dy": 0.0, "dheading": 0.0}

        if self.prev_gray is not None and self.prev_pts is not None:
            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pts, None, **self.lk_params
            )
            good_old = self.prev_pts[status == 1]
            good_new = next_pts[status == 1]

            if len(good_old) > 10:
                displacement = good_new - good_old
                mean_disp = np.mean(displacement, axis=0)
                dx, dy = mean_disp.flatten()

                result["moved"] = True
                result["dx"] = dx
                result["dy"] = dy

                self.position += mean_disp.flatten() * 0.001

        keypoints = self.fast.detect(gray, None)
        if keypoints:
            self.prev_pts = np.array(
                [kp.pt for kp in keypoints], dtype=np.float32
            ).reshape(-1, 1, 2)
        else:
            self.prev_pts = None

        self.prev_gray = gray
        return result
