import numpy as np
import cv2
import json
from pathlib import Path
from ...system.logger import log


class CameraCalibration:
    def __init__(self, chessboard=(9, 6), square_size_mm=25):
        self.chessboard = chessboard
        self.square_size = square_size_mm
        self.mtx = None
        self.dist = None
        self.new_mtx = None
        self.roi = None

    def calibrate(self, images: list):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((self.chessboard[0] * self.chessboard[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard[0], 0:self.chessboard[1]].T.reshape(-1, 2)
        objp *= self.square_size

        objpoints, imgpoints = [], []
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard, None)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

        if len(objpoints) > 0:
            ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
            h, w = gray.shape
            self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w, h), 1, (w, h)
            )
            log.info(f"Camera calibrated: RMS={ret:.4f}")

    def undistort(self, img):
        if self.mtx is None or self.dist is None:
            return img
        return cv2.undistort(img, self.mtx, self.dist, None, self.new_mtx)

    def save(self, path="config/calibration/camera_calib.json"):
        Path(path).parent.mkdir(exist_ok=True)
        data = {
            "mtx": self.mtx.tolist() if self.mtx is not None else None,
            "dist": self.dist.tolist() if self.dist is not None else None,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load(self, path="config/calibration/camera_calib.json"):
        p = Path(path)
        if p.exists():
            with open(p) as f:
                data = json.load(f)
            self.mtx = np.array(data["mtx"])
            self.dist = np.array(data["dist"])
            return True
        return False
