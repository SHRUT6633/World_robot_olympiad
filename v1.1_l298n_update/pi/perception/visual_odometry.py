import cv2
import numpy as np


class VisualOdometry:
    def __init__(self, focal=300, pp=(320, 240)):
        self.focal = focal
        self.pp = pp
        self._prev_kp = None
        self._prev_des = None

    def estimate_motion(self, img):
        orb = cv2.ORB_create(nfeatures=500)
        kp, des = orb.detectAndCompute(img, None)
        if self._prev_kp is None or self._prev_des is None:
            self._prev_kp, self._prev_des = kp, des
            return None
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self._prev_des, des)
        if len(matches) < 8:
            self._prev_kp, self._prev_des = kp, des
            return None
        src_pts = np.float32([self._prev_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        E, mask = cv2.findEssentialMat(src_pts, dst_pts, focal=self.focal, pp=self.pp, method=cv2.RANSAC)
        if E is None:
            self._prev_kp, self._prev_des = kp, des
            return None
        _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, focal=self.focal, pp=self.pp)
        self._prev_kp, self._prev_des = kp, des
        return {"R": R, "t": t}
