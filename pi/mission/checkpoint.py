import math
from ..system.logger import log


class CheckpointManager:
    # CheckpointManager maintains a list of ordered (x, y) waypoints that the
    # robot must visit sequentially.  Each checkpoint has a reach radius
    # (threshold).  The planner and controller use next_target() to obtain
    # the current goal.  This is essential for track navigation: the global
    # planner generates checkpoints, and the controller drives toward each
    # one until all are reached (end of lap or parking).

    def __init__(self):
        self.checkpoints = []    # List of dicts: {x, y, threshold, reached}.
        self._current = 0        # Index of the next checkpoint to reach.

    def add(self, x, y, threshold=0.2):
        # Append a new checkpoint at (x, y) in metres.
        # threshold -- Euclidean distance (metres) within which the checkpoint
        # is considered reached.  Decreasing this makes the robot need to get
        # closer; increasing it allows earlier progress to the next point.
        self.checkpoints.append({
            "x": x, "y": y,
            "threshold": threshold,
            "reached": False,
        })

    def check_reached(self, robot_x, robot_y):
        # Given the robot's current position (robot_x, robot_y), check if the
        # current target checkpoint has been reached.  If so, mark it reached
        # and advance to the next.  Returns True if a checkpoint was just
        # completed, False otherwise.
        if self._current >= len(self.checkpoints):
            return False
        cp = self.checkpoints[self._current]
        dist = math.sqrt((robot_x - cp["x"])**2 + (robot_y - cp["y"])**2)
        if dist < cp["threshold"] and not cp["reached"]:
            cp["reached"] = True
            self._current += 1
            log.info(f"Checkpoint {self._current}/{len(self.checkpoints)} reached")
            return True
        return False

    def next_target(self):
        # Returns the (x, y) of the next un-reached checkpoint, or None if
        # all checkpoints have been completed.
        if self._current < len(self.checkpoints):
            cp = self.checkpoints[self._current]
            return cp["x"], cp["y"]
        return None

    @property
    def all_reached(self):
        # True when every checkpoint has been visited.
        return self._current >= len(self.checkpoints)

    def reset(self):
        # Reset progress so the robot can start a new lap.
        self._current = 0
        for cp in self.checkpoints:
            cp["reached"] = False
