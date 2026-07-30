# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/race_strategy.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Race strategy decision making
# =============================================================================

from ..system.logger import log


class RaceStrategy:
    # RaceStrategy adapts the robot's driving behaviour based on mission
    # progress (lap number), elapsed time, and obstacle density.  It produces
    # a speed_factor that the controller uses to scale the target velocity.
    # The goal is to be cautious on early laps (exploration) and aggressive
    # on later laps (racing), while also slowing down when many obstacles
    # are present.

    def __init__(self):
        # mode can be: "exploration", "normal", "racing", or "cautious".
        self.mode = "normal"

    def update(self, lap, elapsed, obstacles):
        # Called periodically with:
        #   lap       -- current lap number (0 = not yet completed any).
        #   elapsed   -- elapsed mission time in seconds.
        #   obstacles -- number of obstacles recently detected (integer).
        #
        # Priority logic:
        #   1. If obstacle count > 3, switch to "cautious" regardless of lap.
        #   2. Otherwise, lap 0 -> "exploration", lap >= 1 -> "racing".
        #
        # This means obstacles always take precedence over the lap-based
        # mode.
        if lap == 0:
            self.mode = "exploration"
        elif lap >= 1:
            self.mode = "racing"
        if obstacles > 3:
            self.mode = "cautious"

    @property
    def speed_factor(self):
        # Returns a multiplier [0-1] that scales the robot's commanded speed.
        #   exploration: 0.6 -- slow, careful mapping of the track.
        #   normal:      0.8 -- default moderate speed.
        #   racing:      1.0 -- full speed, optimal line.
        #   cautious:    0.5 -- slow down for obstacles.
        factors = {
            "exploration": 0.6,
            "normal": 0.8,
            "racing": 1.0,
            "cautious": 0.5,
        }
        return factors.get(self.mode, 0.7)
