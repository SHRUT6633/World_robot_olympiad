# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/lap_counter.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Lap counting logic
# =============================================================================

from ..system.logger import log


class LapCounter:
    # ── LAP TRACKER ───────────────────────────────────────────────────
    #
    # Tracks how many laps the robot has completed and determines when
    # all laps are done (transition to parking).
    #
    # WHY a dedicated class instead of an int?
    #   - Encapsulates the debounce logic (edge detection).
    #   - Provides `finished` / `remaining` properties that are
    #     semantically clear in the mission planner.
    #   - Easy to swap the detection mechanism (line sensor vs. visual
    #     checkpoint) without changing the rest of the code.
    #
    # HOW LAPS ARE COUNTED:
    #   1. The mission planner detects a finish-line crossing via the
    #      camera (visual line) or a checkpoint ID.
    #   2. It calls mark_crossing() — the counter increments once.
    #   3. The debounce flag (_crossed) prevents multiple increments
    #      from the same crossing event.
    #   4. The planner calls reset_crossing() after the robot has moved
    #      well past the line, re-arming the counter for the next lap.
    #   5. When finished == True, the state machine triggers the
    #      LAP_FINISHED → PARK_APPROACH transition.
    #
    # STATE MACHINE INTEGRATION:
    #   The mission planner checks lap_counter.finished each tick.
    #   When it becomes True, the condition for FORWARD → LAP_FINISHED
    #   fires, and the robot stops racing and begins parking.

    def __init__(self, total_laps=2):
        # total_laps — mission requirement (default 2 for WRO 2026).
        #
        # WHY configurable?
        #   - Practice mode: total_laps=1 for quick testing.
        #   - Competition format may change (some years require 3 laps).
        #   - Robot can adjust based on referee input.
        self.total_laps = total_laps
        self.current_lap = 0      # 0 = no laps completed yet.
                                  # Incremented to 1 after first crossing.
        self._crossed = False     # ── Debounce flag ──
                                  #   True  = currently in a crossing event.
                                  #   False = ready to detect next crossing.
                                  #
                                  # WHY edge detection?
                                  #   The finish line may be visible for
                                  #   multiple camera frames as the robot
                                  #   drives over it.  Without debounce,
                                  #   one physical crossing would produce
                                  #   many increments and break the count.

    def mark_crossing(self):
        # Called by the mission planner when the finish line is detected.
        #
        # The `if not self._crossed` guard is the leading-edge detector:
        # it only fires on the FIRST detection.  Subsequent frames where
        # the line is still visible are ignored until reset_crossing().
        #
        # WHY log every crossing?
        #   Critical for post-race analysis: if the robot mis-counts laps,
        #   the log shows exactly when the crossing was registered.
        if not self._crossed:
            self._crossed = True
            self.current_lap += 1
            log.info(f"Lap {self.current_lap}/{self.total_laps}")

    def reset_crossing(self):
        # Re-arms the debounce flag.
        #
        # WHEN to call this:
        #   After the robot has travelled far enough past the finish line
        #   that it's safe to detect the next crossing.  If called too
        #   early (while still on the line), it immediately re-detects
        #   and double-counts.
        #
        # TRICK: The mission planner can delay reset_crossing() by a
        # distance threshold (e.g. 0.5 m past the line) to create a
        # "debounce zone".
        self._crossed = False

    # ── Convenience properties ───────────────────────────────────────
    @property
    def finished(self):
        # True → all laps are completed.  The state machine should
        # transition the robot to the parking phase.
        return self.current_lap >= self.total_laps

    @property
    def remaining(self):
        # How many laps the robot still needs to complete.
        # Used by the race strategy to decide speed mode:
        #   remaining == 0 → prepare to park.
        #   remaining  > 0 → keep racing.
        # Clamped to 0 to avoid negative numbers if current_lap somehow
        # exceeds total_laps (shouldn't happen, but defensive).
        return max(0, self.total_laps - self.current_lap)
