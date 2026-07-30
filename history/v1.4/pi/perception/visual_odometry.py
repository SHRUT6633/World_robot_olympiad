import cv2
import numpy as np


class VisualOdometry:
    # VisualOdometry estimates the robot's relative motion between consecutive
    # camera frames using ORB feature matching and the essential matrix.
    # It provides a 3D rotation R and translation t which can be fused with
    # IMU and wheel odometry in the UKF sensor fusion module.
    # This is a lightweight monocular VO suitable for indoor track scenarios.

    def __init__(self, focal=300, pp=(320, 240)):
        # focal -- camera focal length in pixels.  This is a calibration
        # parameter that depends on the camera's FOV and resolution.
        # If the focal length is wrong, the recovered translation scale will
        # be incorrect (monocular scale ambiguity).  Default 300 is a guess
        # for a typical 640x480 webcam-like setup.
        # pp -- principal point (cx, cy) in pixels, typically the image centre.
        self.focal = focal
        self.pp = pp

        # Store keypoints and descriptors from the previous frame for matching.
        self._prev_kp = None
        self._prev_des = None

    def estimate_motion(self, img):
        # img -- current BGR camera frame.
        # Returns a dict {"R": 3x3 rotation matrix, "t": 3x1 translation vector}
        # or None if motion cannot be estimated (first frame, too few matches,
        # or essential matrix decomposition fails).

        # Extract ORB features (up to 500 keypoints) from the current frame.
        orb = cv2.ORB_create(nfeatures=500)
        kp, des = orb.detectAndCompute(img, None)

        # First frame -- nothing to match against yet.
        if self._prev_kp is None or self._prev_des is None:
            self._prev_kp, self._prev_des = kp, des
            return None

        # Brute-force Hamming-distance matcher with cross-check to filter
        # spurious matches.
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self._prev_des, des)

        # The 8-point algorithm needs at least 8 good matches.
        # If fewer are found, skip this frame and reset the reference.
        if len(matches) < 8:
            self._prev_kp, self._prev_des = kp, des
            return None

        # Extract matched point coordinates.
        src_pts = np.float32(
            [self._prev_kp[m.queryIdx].pt for m in matches]
        ).reshape(-1, 1, 2)
        dst_pts = np.float32([kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Compute the essential matrix from the matched points.
        # RANSAC rejects outliers automatically.
        E, mask = cv2.findEssentialMat(
            src_pts, dst_pts, focal=self.focal, pp=self.pp, method=cv2.RANSAC
        )

        if E is None:
            self._prev_kp, self._prev_des = kp, des
            return None

        # Decompose the essential matrix into rotation R and translation t.
        # t is up to scale (monocular) -- the true scale must come from
        # another sensor (e.g. ToF or wheel odometry).
        _, R, t, _ = cv2.recoverPose(
            E, src_pts, dst_pts, focal=self.focal, pp=self.pp
        )

        # Update reference for the next frame.
        self._prev_kp, self._prev_des = kp, des

        return {"R": R, "t": t}
