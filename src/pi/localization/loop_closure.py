# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/localization/loop_closure.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Loop closure detection for SLAM pose graph optimisation
# =============================================================================

import numpy as np
from ..system.logger import log


class LoopClosureDetector:
    # ──────────────────────────────────────────────────────────────────
    # Detects when the robot revisits a previously seen location
    # (loop closure).  This is used to correct drift in SLAM / pose
    # graph optimisation.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self, distance_threshold=0.3, angle_threshold=15):
        # distance_threshold – Euclidean distance (metres) below which
        #   two poses are considered a potential match.
        self.dist_thresh = distance_threshold

        # angle_threshold – heading difference (degrees, internally
        #   converted to radians) that must also be below the limit.
        self.angle_thresh = np.radians(angle_threshold)

    def detect(self, current_pose, trajectory):
        # current_pose : (x, y, heading) of the robot *now*.
        # trajectory   : list of (x, y, heading) tuples visited earlier.
        # Returns the *index* of the first matching pose, or None.

        cx, cy, cheading = current_pose

        # Brute-force nearest-neighbour scan over the full trajectory.
        for i, (x, y, heading) in enumerate(trajectory):
            d = np.sqrt((cx - x)**2 + (cy - y)**2)          # Euclidean distance
            a = abs(cheading - heading)                     # absolute heading diff

            # Both must be below their respective thresholds.
            if d < self.dist_thresh and a < self.angle_thresh:
                return i                                     # matched at trajectory[i]

        return None  # no loop closure detected

# ── What happens if you change key values? ─────────────────────────
# * distance_threshold  ↑ more matches → more constraints (but more
#   false positives); ↓ fewer matches → risk of missed closures.
# * angle_threshold     ↑ wider heading tolerance; ↓ stricter match.
# * Changing the loop (e.g. using cosine-similarity on descriptors
#   instead of raw distance) would make detection appearance-aware.
# ────────────────────────────────────────────────────────────────────
