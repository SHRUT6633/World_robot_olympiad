# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/camera/calibration.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Camera intrinsic calibration
# =============================================================================

import numpy as np
import cv2
import json
from pathlib import Path
from ...system.logger import log


class CameraCalibration:
    """
    Camera intrinsic calibration using a printed chessboard pattern.

    Physical meaning:
      A camera lens introduces two types of distortion:
        1. Radial distortion  — straight lines appear curved (barrel/pincushion).
        2. Tangential distortion — the lens is not perfectly parallel to the
           image sensor plane.

      Calibration computes:
        - mtx (camera matrix):      [[fx,  0, cx],
                                      [ 0, fy, cy],
                                      [ 0,  0,  1]]
          where (fx, fy) = focal length in pixels, (cx, cy) = optical centre.
        - dist (distortion coeffs): [k1, k2, p1, p2, k3] radial & tangential.

      The robot uses these to undistort images before processing. Without
      calibration, lines near the image edges appear bent and position
      estimates from AprilTags would be inaccurate.

    Chessboard pattern:
      default (9, 6) means 9 inner corners per row and 6 per column.
      The board should be printed on a flat surface. Each square should
      be exactly square_size_mm wide (default 25 mm). Larger boards
      give better calibration accuracy.

    Configuration parameters:
      chessboard : tuple (cols, rows)
        Number of INNER corners of the chessboard pattern. A 10×7 board
        of squares produces 9×6 inner corners.
      square_size_mm : float
        Physical side length of one square in millimetres. This sets the
        scale of the world coordinate system. Getting this wrong will
        make distance estimates scale incorrectly.
    """

    def __init__(self, chessboard=(9, 6), square_size_mm=25):
        # (inner columns, inner rows) of the chessboard pattern.
        self.chessboard = chessboard
        # Physical size of one square in mm.
        self.square_size = square_size_mm
        # 3×3 camera intrinsic matrix (focal lengths + optical centre).
        self.mtx = None
        # Distortion coefficients [k1, k2, p1, p2, k3].
        self.dist = None
        # Optimised camera matrix (after undistortion) that accounts for
        # the fact that after undistortion the image may be cropped differently.
        self.new_mtx = None
        # Region of interest (x, y, w, h) inside the undistorted image
        # that contains valid pixels (no black borders from undistortion).
        self.roi = None

    def calibrate(self, images: list):
        """
        Run Zhang's calibration algorithm on a list of chessboard images.

        Algorithm:
          1. For each image, find chessboard corners with sub-pixel accuracy.
          2. Build world-space coordinates (flat Z=0 plane) scaled by
             square_size_mm.
          3. Call cv2.calibrateCamera() which solves for mtx, dist, and
             per-image rotation + translation vectors using least squares.

        Number of images needed:
          At least 10–20 images covering different angles (tilt, pan, skew)
          and distances. If the chessboard is always in the centre, the
          corners will be poorly constrained and calibration will be weak.

        Sub-pixel refinement:
          cv2.cornerSubPix uses an iterative search to find corners at
          sub-pixel accuracy, greatly improving calibration precision.
        """
        # Termination criteria for sub-pixel corner refinement: stop after
        # 30 iterations or when corner movement < 0.001 pixels.
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare world-space coordinates for all inner corners.
        # Shape: (N_corners, 3). All Z=0 (flat chessboard plane).
        objp = np.zeros((self.chessboard[0] * self.chessboard[1], 3), np.float32)
        # mgrid creates a grid of (x, y) coordinates for each corner.
        # reshape(-1, 2) flattens to (N, 2), then we set Z=0.
        objp[:, :2] = np.mgrid[0:self.chessboard[0], 0:self.chessboard[1]].T.reshape(-1, 2)
        # Scale by physical square size to get real-world units (mm).
        objp *= self.square_size

        # Lists: objpoints = 3D points in world space, imgpoints = 2D pixels.
        objpoints, imgpoints = [], []
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find chessboard corners to integer pixel accuracy.
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard, None)
            if ret:
                # Valid chessboard found — store world points.
                objpoints.append(objp)
                # Refine to sub-pixel accuracy.
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

        if len(objpoints) > 0:
            # Run Zhang's calibration algorithm.
            # ret = RMS re-projection error (pixels). Lower is better.
            # rvecs, tvecs = per-image rotation & translation (extrinsics).
            ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
            h, w = gray.shape
            # Compute the optimal new camera matrix that minimises the
            # number of black pixels introduced by undistortion.
            # alpha=1 keeps ALL pixels (black borders visible).
            # alpha=0 crops to the maximum valid region.
            self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w, h), 1, (w, h)
            )
            log.info(f"Camera calibrated: RMS={ret:.4f}")

    def undistort(self, img):
        """
        Apply calibration to remove lens distortion from an image.

        Uses cv2.undistort() with the optimised camera matrix (new_mtx)
        which typically results in a slightly zoomed, cropped image that
        has straight lines mapped correctly.

        If calibration has not been performed (mtx is None), returns the
        original image unchanged.
        """
        if self.mtx is None or self.dist is None:
            return img
        return cv2.undistort(img, self.mtx, self.dist, None, self.new_mtx)

    def save(self, path="config/calibration/camera_calib.json"):
        """
        Save the calibration data (mtx and dist) to a JSON file.

        The path is relative to the workspace root. The parent directory
        is created automatically if it does not exist.

        The saved .json file can be loaded on subsequent runs, avoiding
        re-calibration every time the robot boots.
        """
        Path(path).parent.mkdir(exist_ok=True)
        data = {
            "mtx": self.mtx.tolist() if self.mtx is not None else None,
            "dist": self.dist.tolist() if self.dist is not None else None,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load(self, path="config/calibration/camera_calib.json"):
        """
        Load previously saved calibration data from a JSON file.

        Returns True if the file exists and was loaded successfully,
        False if the file is missing (first run, needs calibration).
        """
        p = Path(path)
        if p.exists():
            with open(p) as f:
                data = json.load(f)
            self.mtx = np.array(data["mtx"])
            self.dist = np.array(data["dist"])
            return True
        return False
