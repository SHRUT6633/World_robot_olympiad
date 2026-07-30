import cv2
import numpy as np


MAGENTA_LOWER = np.array([140, 100, 100])
MAGENTA_UPPER = np.array([170, 255, 255])
MIN_AREA_NEAR = 80
DISTANCE_GATE_MM = 500


class PinkDetect:
    def __init__(self):
        self.kernel = np.ones((3, 3), np.uint8)

    def process(self, frame, distance_to_target_mm=None):
        if distance_to_target_mm is None or distance_to_target_mm > DISTANCE_GATE_MM:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        markers = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA_NEAR:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            markers.append({
                "bbox": (x, y, w, h),
                "center": (x + w // 2, y + h // 2),
                "area": area,
            })

        return markers
