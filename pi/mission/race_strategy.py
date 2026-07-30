# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/race_strategy.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Race strategy decision making
# =============================================================================

from ..system.logger import log


class RaceStrategy:
    # ── ADAPTIVE SPEED MANAGER ───────────────────────────────────────
    #
    # Produces a speed_factor [0–1] that scales the robot's commanded
    # velocity based on mission progress and obstacle density.
    #
    # WHY four modes?
    #   WRO tracks have multiple sections with different demands:
    #     • Lap 1 (exploration) – robot doesn't know the track layout
    #       yet.  Slow speeds give the localisation and mapping modules
    #       time to converge and reduce the risk of crashing on unknown
    #       corners.
    #     • Later laps (racing) – the robot has mapped the track and
    #       can drive the racing line at full speed.
    #     • Obstacle-dense sections (cautious) – regardless of lap, if
    #       many obstacles are present, the robot slows down to avoid
    #       collisions.
    #
    # STATE MACHINE INTEGRATION:
    #   The mission planner calls update() each tick with fresh data.
    #   speed_factor is read by the motor controller to scale target
    #   velocity.  The FORWARD and CORNERING states use this factor.
    #
    # TRACK-SECTION ADAPTATION:
    #   The mission planner could call update() with different obstacle
    #   counts depending on the track section (e.g. high obstacle count
    #   in the "slalom" section triggers cautious mode).  This is how
    #   the robot behaves differently in different sections.

    def __init__(self):
        # mode – one of "exploration", "normal", "racing", "cautious".
        # Default to "normal" until the first update() call.
        self.mode = "normal"

    def update(self, lap, elapsed, obstacles):
        # ── MODE SELECTION ──
        # Priority (highest to lowest):
        #   1. obstacles > 3 → "cautious"  (safety override)
        #   2. lap == 0      → "exploration" (first lap)
        #   3. lap >= 1      → "racing"     (subsequent laps)
        #
        # WHY this priority order?
        #   Obstacle avoidance must override everything else – a crash
        #   ends the mission.  Lap-based modes are secondary: they
        #   optimise performance, not safety.
        #
        # PARAMETERS:
        #   lap       – current lap number (0 = no laps completed).
        #               Provided by LapCounter.current_lap.
        #   elapsed   – mission time in seconds (from the planner's
        #               global timer).  Currently unused but reserved
        #               for future time-based strategies (e.g. "if
        #               time < 30s, race; if running late, go faster").
        #   obstacles – count of obstacles detected in the last N frames.
        #               Provided by the sensor fusion module.
        #               Threshold >3 is somewhat arbitrary – tune based
        #               on track obstacle density.
        if lap == 0:
            # ── EXPLORATION ──
            # First lap: the robot is learning the track.  Drive slowly
            # to build a map and avoid surprises.
            self.mode = "exploration"
        elif lap >= 1:
            # ── RACING ──
            # Subsequent laps: the track is known.  Push to full speed.
            self.mode = "racing"

        if obstacles > 3:
            # ── CAUTIOUS OVERRIDE ──
            # Regardless of lap, if obstacles are dense, slow down.
            # This condition is checked AFTER the lap-based assignment,
            # so it takes precedence (the last assignment wins).
            #
            # WHY >3?  A single obstacle might be a false positive;
            # three or more suggests a real obstacle cluster that needs
            # careful navigation.
            self.mode = "cautious"

    @property
    def speed_factor(self):
        # Returns a multiplier [0–1] for the robot's target speed.
        #
        # MODE → FACTOR TABLE:
        #   exploration  0.6  – Slow (60% of max speed).
        #                        Gives time for SLAM/convergence.
        #   normal       0.8  – Moderate (80%).  Default fallback.
        #   racing       1.0  – Full speed.  Only used on known track.
        #   cautious     0.5  – Slow (50%).  For obstacle-dense zones.
        #
        # WHY these specific values?
        #   - 1.0: maximum possible speed the robot can safely maintain
        #     on the track.
        #   - 0.6: slow enough that a sudden obstacle detection gives
        #     enough stopping distance.
        #   - 0.5: extremely cautious – the robot can stop almost
        #     instantly if needed.
        #   - 0.8: a safe default between the two extremes.
        #
        # FALLBACK: if mode is somehow invalid (e.g. None), return 0.7
        # as a safe intermediate value.
        factors = {
            "exploration": 0.6,
            "normal": 0.8,
            "racing": 1.0,
            "cautious": 0.5,
        }
        return factors.get(self.mode, 0.7)
