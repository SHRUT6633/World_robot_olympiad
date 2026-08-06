import time
from ..system.logger import log


class LapCounter:
    def __init__(self, total_laps=3):
        self.total_laps = total_laps
        self.current_lap = 0
        self._crossed = False
        self._last_crossing_time = 0

    def mark_crossing(self):
        now = time.monotonic()
        if not self._crossed and (now - self._last_crossing_time) > 0.5:
            self._crossed = True
            self._last_crossing_time = now
            self.current_lap += 1
            log.info(f"Lap {self.current_lap}/{self.total_laps}")

    def reset_crossing(self):
        self._crossed = False

    @property
    def finished(self):
        return self.current_lap >= self.total_laps

    @property
    def remaining(self):
        return max(0, self.total_laps - self.current_lap)
