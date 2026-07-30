from ..system.logger import log


class LapCounter:
    # LapCounter tracks the current lap number and detects finish-line
    # crossings.  It uses a simple edge-detection approach: the external
    # code calls mark_crossing() when a line-crossing is detected (e.g.
    # via a visual line sensor or a start/finish checkpoint), and the
    # counter increments.  The robot uses this to decide when to switch
    # from "racing" to "parking" and to update strategy.

    def __init__(self, total_laps=2):
        # total_laps -- number of laps the mission requires (default 2).
        # Changing this adjusts how many times the robot must cross the
        # finish line before it parks.
        self.total_laps = total_laps
        self.current_lap = 0      # 0 means haven't completed any lap yet.
        self._crossed = False     # Debounce flag: prevents double-counting.

    def mark_crossing(self):
        # Call this when the finish line is crossed.
        # The debounce ensures only one lap increment per crossing event.
        if not self._crossed:
            self._crossed = True
            self.current_lap += 1
            log.info(f"Lap {self.current_lap}/{self.total_laps}")

    def reset_crossing(self):
        # Call this after the robot has moved past the line to re-arm
        # the counter for the next crossing.
        self._crossed = False

    @property
    def finished(self):
        # True if the robot has completed all required laps.
        return self.current_lap >= self.total_laps

    @property
    def remaining(self):
        # How many laps are still left to complete (never negative).
        return max(0, self.total_laps - self.current_lap)
