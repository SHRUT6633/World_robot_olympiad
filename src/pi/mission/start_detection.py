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
    # ── START-ZONE DETECTOR ──────────────────────────────────────────
    #
    # Detects whether the robot has returned to its starting location
    # by template-matching the current camera view against a stored
    # image of the start area.
    #
    # WHY template matching instead of a line sensor?
    #   The start zone may have distinctive visual features (logos,
    #   markings, WRO signage) that are more robust than a simple line.
    #   Template matching is simple to implement and requires no
    #   training data.
    #
    # WHEN is this used?
    #   After all laps are complete, the robot navigates back to the
    #   start zone to park.  The StartDetector confirms "you are at the
    #   start" so the parking sequence can begin.
    #
    # STATE MACHINE TRANSITION:
    #   LAP_FINISHED → PARK_APPROACH (triggered when detector returns True
    #   and the robot is near the expected parking area).
    #
    # LIMITATIONS (and why they're acceptable):
    #   - TM_CCOEFF_NORMED is scale- and rotation-sensitive.  If the
    #     robot returns at a different yaw angle or distance, the match
    #     may fail.  In WRO 2026 the parking approach is controlled,
    #     so the robot should arrive at roughly the same pose every time.
    #   - Lighting changes between the template capture and the return
    #     can reduce correlation.  The threshold (0.7) provides some
    #     tolerance.

    def __init__(self):
        # ── Internal state ──
        self._template = None   # Greyscale reference image of the start zone.
                                # Set once via set_template() during
                                # robot initialisation (robot is positioned
                                # at the start line facing forward).
                                #
                                # WHY greyscale?
                                #   Colour adds noise and doubles
                                #   computation.  The shape/structure is
                                #   sufficient for matching.
        self._threshold = 0.7   # Minimum normalised cross-correlation score.
                                # Range: [0, 1].
                                #
                                # TUNING GUIDE:
                                #   0.9+ → Very strict (only matches almost
                                #           identical view).  Risk: false
                                #           negatives if lighting changes.
                                #   0.5  → Lenient (matches even partially
                                #           similar scenes).  Risk: false
                                #           positives.
                                #   0.7  → Reasonable balance for indoor
                                #           competition environments.

    def set_template(self, img):
        # Capture the reference template from the current camera frame.
        # This is called once during setup(), after the robot is placed
        # at the starting position.
        #
        # The template is stored in greyscale to speed up matching and
        # reduce sensitivity to colour variations.
        #
        # WHY no preprocessing (histogram equalisation, etc.)?
        #   Keeping it simple reduces CPU load on the Raspberry Pi.
        #   If lighting varies wildly, add cv2.equalizeHist() here and
        #   in detect().
        self._template = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def detect(self, img):
        # Returns True if the current camera feed matches the stored
        # template above the threshold.
        #
        # ── ALGORITHM ──
        #   1. Quick-reject if no template or no image.
        #   2. Convert current frame to greyscale.
        #   3. Run normalised cross-correlation (TM_CCOEFF_NORMED).
        #      This slides the template over the image and computes
        #      the correlation coefficient at each position.
        #   4. Take the maximum coefficient (best match location).
        #   5. Compare to threshold.
        #
        # ── PERFORMANCE ──
        #   cv2.matchTemplate is O(W*H*w*h) where W,H = image size and
        #   w,h = template size.  For a 640×480 image with a 200×150
        #   template this takes ~5–10 ms on a Raspberry Pi 4 — fast
        #   enough for 20+ FPS.
        #
        # ── STATE MACHINE INTEGRATION ──
        #   The mission planner calls this every tick while in
        #   PARK_APPROACH (or after LAP_FINISHED).  When it returns True
        #   for N consecutive frames (to filter noise), the planner
        #   transitions to PARK_ALIGN.

        if self._template is None or img is None:
            # No template captured yet or camera not ready.
            return False

        # Convert current frame to greyscale (must match template colour
        # space for correct correlation).
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Template matching: normalised cross-correlation.
        #   result[x][y] = correlation at position (x, y).
        result = cv2.matchTemplate(gray, self._template, cv2.TM_CCOEFF_NORMED)

        # Extract the maximum correlation value from the result matrix.
        _, max_val, _, _ = cv2.minMaxLoc(result)

        # Return True if the best match exceeds the configured threshold.
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
