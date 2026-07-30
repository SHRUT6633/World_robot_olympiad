import cv2
import numpy as np


class FreeSpaceDetect:
    def __init__(self, block_size=16, threshold=20.0):
        self.block_size = block_size
        self.threshold = threshold

    def process(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]

        texture = cv2.boxFilter(
            sat.astype(np.float32), ddepth=-1,
            ksize=(self.block_size, self.block_size), normalize=False
        )
        smooth_mask = texture < self.threshold

        free_space_pct = np.sum(smooth_mask) / smooth_mask.size * 100.0

        return {
            "free_space_pct": free_space_pct,
            "mask": smooth_mask.astype(np.uint8) * 255,
            "drivable": free_space_pct > 40.0,
        }
