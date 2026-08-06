# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/checkpoint.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Checkpoint detection and management
# =============================================================================

import math
from ..system.logger import log


class CheckpointManager:
    # ── WAYPOINT NAVIGATION MANAGER ──────────────────────────────────
    #
    # Maintains an ordered list of (x, y) waypoints (checkpoints) that
    # the robot must visit sequentially.  Each checkpoint has a reach
    # radius (threshold).  The path tracker calls next_target() each
    # tick to get the current goal.
    #
    # WHY checkpoints instead of a pure reactive controller?
    #   - Pure reactive (wall-following) works on simple tracks but
    #     fails on complex layouts with multiple paths.
    #   - Checkpoints give the robot a global reference: it knows
    #     exactly where to go next.
    #   - The mission planner uses all_reached to detect end-of-lap
    #     and end-of-mission.
    #
    # HOW IT WORKS:
    #   1. A global planner (not in this file) generates a sequence of
    #      (x, y) points along the track centreline.
    #   2. The CheckpointManager stores them in order.
    #   3. Each tick, the path tracker gets next_target() and steers
    #      toward that point.
    #   4. When the robot's position is within `threshold` metres of
    #      the current checkpoint, check_reached() advances to the next.
    #   5. When all checkpoints are reached, all_reached=True signals
    #      the mission planner (e.g. end of lap, or parking complete).
    #
    # STATE MACHINE INTEGRATION:
    #   The mission planner calls check_reached() every tick while in
    #   FORWARD or PARK_APPROACH states.  When all_reached is True,
    #   it triggers a transition (LAP_FINISHED or PARK_ALIGN).
    #
    # ERROR RECOVERY:
    #   If the robot misses a checkpoint (drives past it), the distance
    #   to the missed checkpoint will increase.  The mission planner
    #   can implement a timeout:
    #     if dist to current CP is increasing and > threshold × 3:
    #         mark CP as reached anyway (skip).
    #   This prevents the robot from getting stuck trying to reach an
    #   unreachable checkpoint.

    def __init__(self):
        # checkpoints – ordered list of dicts, each with:
        #     x, y       – target position (metres, world frame).
        #     threshold  – reach radius (metres).
        #     reached    – bool, set True once visited.
        self.checkpoints = []

        # _current – index of the next checkpoint to reach.
        # Starts at 0.  Incremented by check_reached().
        # When _current >= len(checkpoints), all are done.
        self._current = 0

    def add(self, x, y, threshold=0.2):
        # Register a new checkpoint.
        #
        # threshold – Euclidean distance (metres) from the checkpoint
        # within which the robot is considered to have "reached" it.
        #
        # TUNING threshold:
        #   0.1 → Tight: robot must pass almost exactly over the point.
        #         Good for precision (parking).  Risk: may never reach
        #         it due to control error.
        #   0.3 → Loose: robot can be further away.
        #         Good for straights.  Risk: skips corners too early.
        #   0.2 → Balanced default.
        self.checkpoints.append({
            "x": x, "y": y,
            "threshold": threshold,
            "reached": False,
        })

    def check_reached(self, robot_x, robot_y):
        # ── PROGRESS CHECK ──
        # Called every tick with the robot's current estimated position.
        #
        # Returns True if the current target checkpoint was just reached
        # (and the index advanced).  Returns False otherwise.
        #
        # WHY a separate function instead of checking next_target()
        #   externally?  Because we need to atomically "mark reached +
        #   advance index" to avoid race conditions between the check
        #   and the advance.

        # If we've already finished all checkpoints, nothing to check.
        if self._current >= len(self.checkpoints):
            return False

        cp = self.checkpoints[self._current]

        # Euclidean distance from robot to current checkpoint.
        dist = math.sqrt((robot_x - cp["x"])**2 + (robot_y - cp["y"])**2)

        # Check if we're inside the threshold AND haven't already
        # marked this checkpoint as reached (safety double-check).
        if dist < cp["threshold"] and not cp["reached"]:
            cp["reached"] = True
            self._current += 1
            log.info(f"Checkpoint {self._current}/{len(self.checkpoints)} reached")
            return True

        return False

    def next_target(self):
        # Returns the (x, y) of the current target, or None if done.
        #
        # The path tracker uses this to compute the heading error and
        # generates steering commands.
        #
        # WHY return None instead of the last checkpoint?
        #   If the robot has passed all checkpoints, there's no target.
        #   The controller should stop or hold position.
        if self._current < len(self.checkpoints):
            cp = self.checkpoints[self._current]
            return cp["x"], cp["y"]
        return None

    @property
    def all_reached(self):
        # True if every checkpoint has been visited.
        # The mission planner checks this to determine:
        #   - Lap complete → increment lap counter.
        #   - Parking complete → transition to PARK_VERIFY.
        return self._current >= len(self.checkpoints)

    def reset(self):
        # Reset all checkpoints to "unreached".
        #
        # WHY reset?  The same checkpoint set is reused for each lap
        # (the track layout doesn't change).  reset() lets the robot
        # restart the sequence without rebuilding the list.
        #
        # NOTE: The threshold values and positions are preserved — only
        # the reached flags and index are reset.
        self._current = 0
        for cp in self.checkpoints:
            cp["reached"] = False
