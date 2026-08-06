# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/pillar_detector.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Coloured pillar detection via HSV segmentation
# =============================================================================

import cv2
import numpy as np


class PillarDetector:
    def __init__(self, config=None):
        if config is None:
            config = {}

        # --- HSV colour ranges for each pillar colour ---
        # Why HSV over RGB?  HSV separates hue (colour) from saturation and
        # value (brightness).  Under varying indoor / outdoor lighting the hue
        # of a coloured object is far more stable than its RGB triplet, so HSV
        # thresholding produces far fewer missed detections when shadows or
        # glare hit the playing field.

        # Red (first wrap):  H 0–10   (red sits at the 0°/180° wrap boundary)
        cv2_hsv_red_low = config.get("red_lower", (0, 100, 100))
        cv2_hsv_red_high = config.get("red_upper", (10, 255, 255))
        # Red (second wrap): H 170–180  (red straddles hue = 0 in OpenCV)
        cv2_hsv_red_low2 = config.get("red_lower2", (170, 100, 100))
        cv2_hsv_red_high2 = config.get("red_upper2", (180, 255, 255))
        # Green:  H 40–90    (WRO rulebook green, typically bright green pillar)
        cv2_hsv_green_low = config.get("green_lower", (40, 50, 50))
        cv2_hsv_green_high = config.get("green_upper", (90, 255, 255))
        # Magenta / pink:  H 140–170   (WRO rulebook "magenta" markers)
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

        # Convert BGR → HSV once, then apply per-colour inRange masks.
        # This is more efficient than converting per colour.
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Red requires two ranges because the hue circle wraps at 180°.
        mask_red1 = cv2.inRange(hsv, self.red_range1[0], self.red_range1[1])
        mask_red2 = cv2.inRange(hsv, self.red_range2[0], self.red_range2[1])
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, self.green_range[0], self.green_range[1])
        mask_pink = cv2.inRange(hsv, self.pink_range[0], self.pink_range[1])

        # Morphological kernel used to clean up the binary masks.
        base_kernel = np.ones((5, 5), np.uint8)
        results = {}

        for label, mask in [("red", mask_red), ("green", mask_green), ("pink", mask_pink)]:
            # MORPH_OPEN  = erosion → dilation: removes small noise blobs.
            # MORPH_CLOSE = dilation → erosion: fills small holes in the blob.
            # Order: open first to remove pepper noise, then close to fill gaps.
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, base_kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, base_kernel)

            # RETR_EXTERNAL: only the outermost contour — no nested contours.
            # This avoids counting holes inside a pillar as separate objects.
            contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            detections = []
            for cnt in contours:
                area = cv2.contourArea(cnt)

                # --- False-positive rejection: minimum area filter ---
                # Any blob smaller than 200 px² is likely noise (specular
                # reflection, tiny colour patch on the floor, etc.).
                # Tune this upward if the detector sees too many ghosts,
                # or downward if real pillars at long range are being missed.
                if area < 200:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                # Bearing: normalised horizontal offset from image centre.
                # negative = left of centre, positive = right.
                # Unitless (fraction of half-width) so it's independent of
                # camera resolution.
                frame_center_x = frame_bgr.shape[1] // 2
                bearing = (cx - frame_center_x) / frame_center_x

                # --- Depth estimation via pinhole model ---
                # Using the similar-triangles relation:
                #   distance = (known_height * focal_length) / pixel_height
                # This assumes the pillar stands upright on the ground plane
                # and we see its full height.  The focal length is a rough
                # calibration value (400 px) — for production, read fx from
                # the camera matrix.
                distance_est = self._estimate_distance(h, label)

                detections.append({
                    "x": cx, "y": cy, "w": w, "h": h,
                    "area": area,
                    "bearing": bearing,
                    "distance_mm": distance_est,
                    "pixel_height": h,
                })

            if detections:
                # Sort by area descending so the caller sees the largest
                # (closest / most prominent) pillar first.
                detections.sort(key=lambda d: d["area"], reverse=True)
                results[label] = detections

        self._last_detections = results
        return results

    def _estimate_distance(self, pixel_height, label):
        # Known physical heights from the WRO 2026 rulebook (in mm).
        # Red and green pillars are full-height (~50 mm); pink markers are
        # smaller markers (~20 mm) used for parking bay boundaries.
        expected_heights = {"red": 50.0, "green": 50.0, "pink": 20.0}
        known_height_mm = expected_heights.get(label, 50.0)
        focal_length = 400.0  # Approximate focal length in pixels (640×480).
        if pixel_height < 1:
            return None
        return (known_height_mm * focal_length) / pixel_height

    # --------------------------------------------------------------
    # Convenience queries for the controller / state machine.
    # All return None when the requested pillar is not currently visible.
    # --------------------------------------------------------------
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
