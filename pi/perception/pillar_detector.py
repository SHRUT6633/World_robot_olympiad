import cv2
import numpy as np


class PillarDetector:
    def __init__(self, config=None):
        if config is None:
            config = {}
        cv2_hsv_red_low = config.get("red_lower", (0, 100, 100))
        cv2_hsv_red_high = config.get("red_upper", (10, 255, 255))
        cv2_hsv_red_low2 = config.get("red_lower2", (170, 100, 100))
        cv2_hsv_red_high2 = config.get("red_upper2", (180, 255, 255))
        cv2_hsv_green_low = config.get("green_lower", (40, 50, 50))
        cv2_hsv_green_high = config.get("green_upper", (90, 255, 255))
        cv2_hsv_pink_low = config.get("pink_lower", (140, 100, 50))
        cv2_hsv_pink_high = config.get("pink_upper", (170, 255, 255))
        self.red_range1 = (np.array(cv2_hsv_red_low, dtype=np.uint8),
                           np.array(cv2_hsv_red_high, dtype=np.uint8))
        self.red_range2 = (np.array(cv2_hsv_red_low2, dtype=np.uint8),
                           np.array(cv2_hsv_red_high2, dtype=np.uint8))
        self.green_range = (np.array(cv2_hsv_green_low, dtype=np.uint8),
                            np.array(cv2_hsv_green_high, dtype=np.uint8))
        self.pink_range = (np.array(cv2_hsv_pink_low, dtype=np.uint8),
                           np.array(cv2_hsv_pink_high, dtype=np.uint8))
        self._last_detections = {}

    def detect(self, frame_bgr):
        if frame_bgr is None:
            return {}
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv, self.red_range1[0], self.red_range1[1])
        mask_red2 = cv2.inRange(hsv, self.red_range2[0], self.red_range2[1])
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, self.green_range[0], self.green_range[1])
        mask_pink = cv2.inRange(hsv, self.pink_range[0], self.pink_range[1])
        base_kernel = np.ones((5, 5), np.uint8)
        results = {}
        for label, mask in [("red", mask_red), ("green", mask_green), ("pink", mask_pink)]:
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, base_kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, base_kernel)
            contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 200:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2
                frame_center_x = frame_bgr.shape[1] // 2
                bearing = (cx - frame_center_x) / frame_center_x
                distance_est = self._estimate_distance(h, label)
                detections.append({
                    "x": cx, "y": cy, "w": w, "h": h,
                    "area": area,
                    "bearing": bearing,
                    "distance_mm": distance_est,
                    "pixel_height": h,
                })
            if detections:
                detections.sort(key=lambda d: d["area"], reverse=True)
                results[label] = detections
        self._last_detections = results
        return results

    def _estimate_distance(self, pixel_height, label):
        expected_heights = {"red": 50.0, "green": 50.0, "pink": 20.0}
        known_height_mm = expected_heights.get(label, 50.0)
        focal_length = 400.0
        if pixel_height < 1:
            return None
        return (known_height_mm * focal_length) / pixel_height

    def is_pillar_left(self, label="red"):
        dets = self._last_detections.get(label, [])
        if not dets:
            return None
        return dets[0]["bearing"] < 0

    def is_pillar_right(self, label="red"):
        dets = self._last_detections.get(label, [])
        if not dets:
            return None
        return dets[0]["bearing"] > 0

    def pillar_bearing(self, label="red"):
        dets = self._last_detections.get(label, [])
        if not dets:
            return None
        return dets[0]["bearing"]

    def pillar_distance(self, label="red"):
        dets = self._last_detections.get(label, [])
        if not dets:
            return None
        return dets[0]["distance_mm"]

    def has_pillar(self, label="red"):
        return label in self._last_detections and len(self._last_detections[label]) > 0
