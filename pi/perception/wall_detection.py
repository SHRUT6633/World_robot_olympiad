# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/wall_detection.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Wall detection from ToF distance readings
# =============================================================================

import numpy as np


class WallDetector:
    # WallDetector converts raw Time-of-Flight (ToF) distance readings into
    # boolean flags that indicate whether the robot is close to a wall on its
    # left, right, or front.  The state machine and the obstacle-avoidance /
    # reverse logic modules consume these flags to decide when to turn or
    # reverse.
    #
    # Why ToF instead of a camera for wall detection?
    #   - ToF sensors (VL53L1X etc.) give direct millimetre-distance readings
    #     at up to 50 Hz with low CPU cost — they are not affected by lighting
    #     conditions or floor texture.
    #   - The camera-based methods (optical flow, depth estimation) are used for
    #     longer-range navigation; ToF handles the close-range wall-following
    #     and parking tasks where precise distance is critical.

    def __init__(self, threshold_mm=500):
        # threshold_mm — distance in millimetres below which a wall is
        # considered "detected".  Default 500 mm = 0.5 m.
        # Lower values make the robot more tolerant of nearby obstacles;
        # higher values make it more cautious and trigger avoidance earlier.
        # On a typical WRO field with walls 1–2 m apart, 500 mm gives a
        # reasonable "close enough to react" buffer.
        self.threshold = threshold_mm

    def detect(self, left_dist, right_dist, front_dist):
        # left_dist / right_dist / front_dist — raw ToF sensor readings
        # in millimetres.  Can be None if a sensor is unavailable (e.g.
        # I²C bus error, sensor out of range).
        #
        # Returns a dict: {"left": bool, "right": bool, "front": bool}
        # where True means the wall is closer than threshold_mm.
        #
        # False-positive rejection:
        #   - ToF sensors occasionally report spurious 0 or very large values.
        #     The caller is expected to filter those (e.g. clamp to [30, 4000])
        #     before passing to this detector.
        #   - This detector does no temporal filtering itself; the state
        #     machine on top of it (in reverse_logic / wall_follow) is
        #     expected to add hysteresis (e.g. require 3 consecutive
        #     "wall close" readings before reacting).
        walls = {"left": False, "right": False, "front": False}

        if left_dist is not None and left_dist < self.threshold:
            walls["left"] = True
        if right_dist is not None and right_dist < self.threshold:
            walls["right"] = True
        if front_dist is not None and front_dist < self.threshold:
            walls["front"] = True

        return walls
