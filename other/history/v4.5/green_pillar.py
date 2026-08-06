import cv2
import numpy as np


GREEN_LOWER_HSV = np.array([50, 100, 100])
GREEN_UPPER_HSV = np.array([80, 255, 255])
MIN_AREA = 400
MIN_ASPECT = 1.5


class GreenPillarDetect:
    def __init__(self):
        self.kernel = np.ones((3, 3), np.uint8)

    def process(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_LOWER_HSV, GREEN_UPPER_HSV)
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
            if min(h, w) == 0:
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
                "hue": hsv[int(rect[0][1]), int(rect[0][0]), 0],
            })

        return pillars

    def tune_hsv(self, frame, center_x, center_y):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h = hsv[center_y, center_x, 0]
        s = hsv[center_y, center_x, 1]
        v = hsv[center_y, center_x, 2]
        return {"H": h, "S": s, "V": v}
