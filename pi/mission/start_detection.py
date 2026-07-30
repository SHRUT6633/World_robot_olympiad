# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/start_detection.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Start line detection
# =============================================================================

import cv2
import numpy as np


class StartDetector:
    # ──────────────────────────────────────────────────────────────────
    # Detects whether the robot has returned to its starting location
    # by template-matching the current camera view against a stored
    # image of the start area.
    #
    # Used to trigger end-of-mission behaviour.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        # _template  – greyscale reference image of the start zone.
        self._template = None
        # _threshold – minimum normalised correlation score to accept
        #              a match (0 = no match, 1 = perfect match).
        self._threshold = 0.7

    def set_template(self, img):
        # Store a greyscale version of the provided colour image as
        # the start-zone template.
        self._template = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def detect(self, img):
        # Return True if the current camera image matches the start
        # template above the threshold, False otherwise.

        if self._template is None or img is None:
            return False

        # Convert current frame to greyscale.
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Normalised cross-correlation template matching.
        result = cv2.matchTemplate(gray, self._template, cv2.TM_CCOEFF_NORMED)

        # Get the maximum correlation value.
        _, max_val, _, _ = cv2.minMaxLoc(result)

        # Compare against the threshold.
        return max_val >= self._threshold

# ── What happens if you change key values? ─────────────────────────
# * _threshold  ↑ → harder to trigger return detection (fewer false
#   positives, may miss actual returns).  ↓ → easier to detect but
#   prone to false positives if the environment looks similar.
# * The template is fixed after `set_template`.  If lighting changes
#   dramatically the matching score will drop.
# * cv2.TM_CCOEFF_NORMED is scale- and rotation-*sensitive*.
#   If the robot returns at a different angle or distance the match
#   may fail.  More robust approaches: feature-based matching (ORB /
#   SIFT) or a learned classifier.
# * If you swap to a different method (e.g. cv2.TM_CCORR_NORMED) the
#   score distribution changes and the threshold must be re-tuned.
# ────────────────────────────────────────────────────────────────────
