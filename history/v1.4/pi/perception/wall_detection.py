import numpy as np


class WallDetector:
    # WallDetector converts raw ToF distance readings into boolean flags that
    # indicate whether the robot is close to a wall on its left, right, or
    # front.  The state machine and the obstacle-avoidance / reverse logic
    # modules consume these flags to decide when to turn or reverse.

    def __init__(self, threshold_mm=500):
        # threshold_mm -- distance in millimetres below which a wall is
        # considered "detected".  Default 500 mm = 0.5 m.
        # Lower values make the robot more tolerant of nearby obstacles;
        # higher values make it more cautious and trigger avoidance earlier.
        self.threshold = threshold_mm

    def detect(self, left_dist, right_dist, front_dist):
        # left_dist / right_dist / front_dist -- raw ToF sensor readings
        # (millimetres).  Can be None if a sensor is unavailable.
        # Returns a dict:
        #   {"left": bool, "right": bool, "front": bool}
        # where True means the wall is closer than threshold_mm.
        walls = {"left": False, "right": False, "front": False}

        if left_dist is not None and left_dist < self.threshold:
            walls["left"] = True
        if right_dist is not None and right_dist < self.threshold:
            walls["right"] = True
        if front_dist is not None and front_dist < self.threshold:
            walls["front"] = True

        return walls
