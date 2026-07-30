# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/visual_odometry.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Monocular visual odometry
# =============================================================================

import cv2
import numpy as np


class VisualOdometry:
    # VisualOdometry estimates the robot's relative motion between consecutive
    # camera frames using ORB feature matching and the essential matrix.
    # It provides a 3D rotation R and translation t which can be fused with
    # IMU and wheel odometry in the UKF sensor fusion module.
    #
    # Why monocular VO instead of stereo?
    #   - The robot carries a single forward-facing camera.
    #   - Monocular VO has scale ambiguity (t is unitless), but the UKF fuses
    #     it with wheel encoders that provide metric scale.
    #
    # Camera-to-world coordinate transformation:
    #   The camera frame is: +X right, +Y down, +Z forward (OpenCV convention).
    #   The world/robot frame is: +X forward, +Y left, +Z up (ROS-style).
    #   The recovered R and t are in camera coordinates and must be rotated
    #   by the extrinsic calibration (typically a 90° rotation about X) to
    #   convert to the robot base frame before entering the UKF.

    def __init__(self, focal=300, pp=(320, 240)):
        # focal — camera focal length in pixels.  This is a calibration
        # parameter that depends on the camera's FOV and resolution.
        # If the focal length is wrong, the recovered translation scale will
        # be incorrect (monocular scale ambiguity).  Default 300 is a guess
        # for a typical 640×480 webcam-like setup.  For production, extract
        # fx from the camera calibration matrix.
        # pp — principal point (cx, cy) in pixels, typically the image centre.
        self.focal = focal
        self.pp = pp

        # Store keypoints and descriptors from the previous frame for matching
        # between consecutive frames.
        self._prev_kp = None
        self._prev_des = None

    def estimate_motion(self, img):
        # img — current BGR camera frame.
        # Returns a dict {"R": 3×3 rotation matrix, "t": 3×1 translation vector}
        # or None if motion cannot be estimated (first frame, too few matches,
        # or essential matrix decomposition fails).

        # Step 1 — Extract ORB features (up to 500 keypoints) from the current
        # frame.  ORB is chosen over SIFT/SURF because it is free (no patent)
        # and fast enough for real-time on a Pi.
        orb = cv2.ORB_create(nfeatures=500)
        kp, des = orb.detectAndCompute(img, None)

        # Step 2 — First frame: no previous features to match against yet.
        # Store and wait for the next frame.
        if self._prev_kp is None or self._prev_des is None:
            self._prev_kp, self._prev_des = kp, des
            return None

        # Step 3 — Brute-force Hamming-distance matcher with cross-check.
        # Cross-check ensures symmetry (i matches j AND j matches i), which
        # is a simple but effective way to reject false correspondences.
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self._prev_des, des)

        # Step 4 — The 8-point algorithm needs at least 8 good matches.
        # If fewer are found, the motion estimate would be unreliable, so we
        # skip this frame and reset the reference to the current frame.
        if len(matches) < 8:
            self._prev_kp, self._prev_des = kp, des
            return None

        # Step 5 — Extract matched point coordinates for the essential matrix.
        src_pts = np.float32(
            [self._prev_kp[m.queryIdx].pt for m in matches]
        ).reshape(-1, 1, 2)
        dst_pts = np.float32([kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Step 6 — Compute the essential matrix E using RANSAC.
        # E encodes the relative rotation and translation (up to scale) between
        # the two views.  RANSAC automatically rejects outlier matches.
        E, mask = cv2.findEssentialMat(
            src_pts, dst_pts, focal=self.focal, pp=self.pp, method=cv2.RANSAC
        )

        if E is None:
            # Essential matrix could not be computed — insufficient inliers.
            self._prev_kp, self._prev_des = kp, des
            return None

        # Step 7 — Decompose E into rotation R and translation t.
        # recoverPose uses the cheirality check to choose the correct
        # orientation among the four mathematical solutions of E decomposition.
        # t is unitless (monocular scale ambiguity); the true metric scale
        # must come from another sensor (e.g. ToF or wheel odometry) via the
        # UKF sensor fusion.
        _, R, t, _ = cv2.recoverPose(
            E, src_pts, dst_pts, focal=self.focal, pp=self.pp
        )

        # Step 8 — Update reference for the next frame.
        self._prev_kp, self._prev_des = kp, des

        return {"R": R, "t": t}
