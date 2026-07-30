import cv2
import numpy as np
from ...system.logger import log


class CameraPipeline:
    def __init__(self):
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def undistort(self, img, mtx, dist):
        if mtx is None:
            return img
        return cv2.undistort(img, mtx, dist)

    def perspective_transform(self, img, src_pts, dst_pts, size):
        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        return cv2.warpPerspective(img, H, size)

    def auto_exposure(self, img, target_brightness=128):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mean = np.mean(gray)
        gain = target_brightness / (mean + 1e-6)
        return np.clip(img * gain, 0, 255).astype(np.uint8)

    def white_balance(self, img):
        result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        avg_a = np.mean(result[:, :, 1])
        avg_b = np.mean(result[:, :, 2])
        result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
        return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)

    def gamma_correction(self, img, gamma=1.2):
        inv = 1.0 / gamma
        table = np.array([(i / 255.0) ** inv * 255 for i in range(256)]).astype("uint8")
        return cv2.LUT(img, table)

    def apply_clahe(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def motion_blur_compensate(self, img, kernel_size=3):
        if kernel_size < 2:
            return img
        kernel = np.ones((kernel_size, kernel_size), np.float32) / (kernel_size * kernel_size)
        return cv2.filter2D(img, -1, kernel)

    def process(self, img, mtx=None, dist=None, src_pts=None, dst_pts=None, warp_size=None):
        if img is None:
            return None
        img = self.undistort(img, mtx, dist)
        if src_pts is not None and dst_pts is not None and warp_size is not None:
            img = self.perspective_transform(img, src_pts, dst_pts, warp_size)
        img = self.auto_exposure(img)
        img = self.white_balance(img)
        img = self.gamma_correction(img)
        img = self.apply_clahe(img)
        return img
