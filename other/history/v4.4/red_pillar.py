import cv2
import numpy as np


RED_LOWER = np.array([200, 20, 30])
RED_UPPER = np.array([255, 80, 80])
MIN_AREA = 400
MIN_ASPECT = 1.5


class RedPillarDetect:
    def __init__(self):
        self.kernel = np.ones((3, 3), np.uint8)

    def process(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(rgb, RED_LOWER, RED_UPPER)
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pillars = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue

            rect = cv2.minAreaRect(cnt)
            h, w = rect[1]
            if w == 0:
                continue
            aspect = max(h, w) / min(h, w)
            if aspect < MIN_ASPECT:
                continue

            box = cv2.boxPoints(rect)
            box = box.astype(np.int32)

            pillars.append({
                "box": box,
                "center": (int(rect[0][0]), int(rect[0][1])),
                "area": area,
                "aspect": aspect,
            })

        return pillars
