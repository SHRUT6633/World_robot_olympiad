import cv2
import numpy as np


class ObjectDetector:
    # ------------------------------------------------------------------
    # 1) Constructor: initialises the detector.
    #    _bboxes – a protected instance variable that *could* be used to
    #              store a history of detected bounding boxes (currently
    #              it is created but never read – a placeholder for future
    #              temporal filtering or tracking).
    # ------------------------------------------------------------------
    def __init__(self):
        self._bboxes = []

    # ------------------------------------------------------------------
    # 2) detect_contours(img, min_area=500) -> list[dict]
    #
    #    The core detection pipeline:
    #
    #    Step 1 – Guard: if img is None (e.g. camera read failed), return
    #             an empty list immediately so the robot does not crash.
    #
    #    Step 2 – Grayscale conversion: colour is irrelevant for simple
    #             blob detection; single-channel is faster.
    #
    #    Step 3 – Gaussian blur (5×5 kernel): reduces high-frequency noise
    #             that would otherwise produce spurious contours.
    #
    #    Step 4 – Binary threshold (THRESH_BINARY_INV, threshold = 60):
    #             Pixels darker than 60 become white (foreground); lighter
    #             become black (background).  The inverted mode is useful
    #             when objects are dark on a bright background.  Changing
    #             the threshold (60) makes detection more or less
    #             sensitive to shadows / lighting.
    #
    #    Step 5 – findContours: extracts external contours from the binary
    #             image (RETR_EXTERNAL = no nested contours, saves time).
    #
    #    Step 6 – Filter by area: only keep contours > min_area (default
    #             500 px²).  This discards tiny noise blobs.  If you
    #             *lower* min_area the detector catches more (and smaller)
    #             objects but also more false positives.
    #
    #    Step 7 – For each qualifying contour, compute the bounding
    #             rectangle and centre point, then dump everything into
    #             a list of dictionaries.
    #
    #    Return value:  [ {"bbox": (x,y,w,h), "area": a, "cx": cx, "cy": cy}, ... ]
    #
    #    Connection to the system:
    #      - The returned list is consumed by
    #        DynamicObstacleAvoidance.avoid() so the robot can steer
    #        away from detected obstacles.
    #      - "cx", "cy" can be used for object tracking or centring
    #        behaviour.
    # ------------------------------------------------------------------
    def detect_contours(self, img, min_area=500):
        if img is None:
            return []
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                objects.append({"bbox": (x, y, w, h), "area": area, "cx": x + w//2, "cy": y + h//2})
        return objects
