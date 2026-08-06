# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/landmark_detection.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Template-based landmark recognition
# =============================================================================

# =============================================================================
# landmark_detection.py — Template-based landmark recognition
# =============================================================================
# Pre-register visual templates (e.g. "start_zone", "loading_bay") and then
# detect() counts how many ORB feature matches exist between the current
# camera view and each template.  If the count exceeds a threshold the
# landmark is considered "present".
#
# This is typically used in WRO (World Robot Olympiad) tasks where the robot
# must recognise coloured signs, numbered markers, or zone indicators.
# =============================================================================

import cv2                                  # OpenCV
import numpy as np                          # (used indirectly)


class LandmarkDetector:
    """
    Multi-template landmark detector based on ORB feature matching.

    Usage
    -----
        detector = LandmarkDetector()
        detector.add_template("start_zone", cv2.imread("start.png"))
        detector.add_template("loading_bay", cv2.imread("loading.png"))

        frame = cv2.imread("current_view.jpg")
        found = detector.detect(frame, threshold=30)
        print(found)   # e.g. {"start_zone": 45, "loading_bay": 12}

    Attributes
    ----------
    _templates : dict
        {name: (keypoints, descriptor)} — the reference data for each landmark.
    """

    def __init__(self):
        self._templates = {}                # name → (kp, des) tuple

    # ------------------------------------------------------------------
    # add_template — Register a new reference image as a landmark
    # ------------------------------------------------------------------
    def add_template(self, name, img):
        """
        Extract ORB features from *img* and store under *name*.

        The template image should be as close as possible to how the landmark
        will appear in the live camera feed (same scale / orientation helps).

        Parameters
        ----------
        name : str        — Landmark identifier.
        img  : np.ndarray — Template image (BGR or grayscale).

        Notes
        -----
        - Each call creates a new ORB instance (no shared state).
        - If you re-add an existing name the old template is overwritten.
        - A template with no features (des is None) will be gracefully skipped
          during detect().
        """
        kp, des = cv2.ORB_create().detectAndCompute(img, None)
        self._templates[name] = (kp, des)

    # ------------------------------------------------------------------
    # detect — Compare a query image against all registered templates
    # ------------------------------------------------------------------
    def detect(self, img, threshold=30):
        """
        For every registered template, count how many ORB matches survive
        a brute-force matcher (cross-checked).  Templates with more matches
        than *threshold* are reported.

        Parameters
        ----------
        img       : np.ndarray — Current camera frame (BGR).
        threshold : int
            Minimum number of good matches needed to declare a landmark found.
            Lower  → more false positives (noise recognised as landmarks).
            Higher → more false negatives (real landmarks missed).

        Returns
        -------
        dict
            {name: match_count} for every template exceeding *threshold*.
            Empty dict if no landmarks are recognised.

        What happens if you change threshold?
        - threshold=10  → very sensitive; may detect "landmarks" in clutter.
        - threshold=80  → very strict; only strong, unambiguous matches pass.
        - threshold=0   → returns all templates with at least 1 match (noisy).

        Performance note
        ----------------
        A new ORB is created per call so that the detector doesn't carry
        state across frames.  For high-FPS use you could cache it or share
        the ORB instance from feature_matching.py.
        """
        orb = cv2.ORB_create()                # Fresh detector each frame
        kp, des = orb.detectAndCompute(img, None)

        if des is None:                       # No features in the frame
            return {}                         # Nothing to match against

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        results = {}
        for name, (_, tdes) in self._templates.items():
            if tdes is None:                  # Template had no features
                continue                      # Cannot match — skip gracefully

            matches = bf.match(tdes, des)     # Cross-checked matches
            if len(matches) > threshold:
                results[name] = len(matches)  # Report match count

        return results
