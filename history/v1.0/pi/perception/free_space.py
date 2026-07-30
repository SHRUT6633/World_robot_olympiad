import cv2
import numpy as np


class FreeSpaceDetector:
    def __init__(self, sobel_thresh=(30, 150)):
        self.sobel_thresh = sobel_thresh

    def detect(self, img):
        if img is None:
            return None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)
        mag = np.sqrt(sobelx**2 + sobely**2)
        _, free = cv2.threshold(mag.astype(np.uint8), self.sobel_thresh[1], 255, cv2.THRESH_BINARY_INV)
        return free
