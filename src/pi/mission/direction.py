# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/direction.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Direction/driving direction detection
# =============================================================================

class DirectionDetector:
    # ── TURNING DIRECTION CLASSIFIER ─────────────────────────────────
    #
    # Infers the robot's turning direction from the instantaneous
    # heading rate (angular velocity, rad/s).
    #
    # WHY a separate class instead of a simple if/else in the planner?
    #   • Centralises the sign convention (positive = CCW = left turn).
    #   • Can be extended with hysteresis, filtering, or deadband
    #     without modifying the mission planner.
    #   • Provides a clear property for the state machine to query.
    #
    # STATE MACHINE INTEGRATION:
    #   The mission planner calls update() every tick with the IMU
    #   gyro Z-axis reading.  It then uses the direction property to
    #   decide state transitions:
    #     - "left"/"right" and speed is low → CORNERING.
    #     - "forward" and all checkpoints passed → LAP_FINISHED.
    #   Also used to trigger LAP_FINISHED → PARK_APPROACH transition
    #   (ensure robot is going straight when approaching the start zone).
    #
    # RACE STRATEGY INTEGRATION:
    #   The race strategy can adapt speed based on direction:
    #     - Entering a turn (left/right) → reduce speed_factor.
    #     - Exiting a turn (forward) → restore speed_factor.
    #   This gives section-specific behaviour.

    def __init__(self):
        # Internal direction string, default "forward".
        self._direction = "forward"

    def update(self, heading_rate):
        # Classify the current turn direction from the heading rate.
        #
        # heading_rate – angular velocity from the IMU gyroscope,
        #                Z-axis (rad/s).  Positive = counter-clockwise
        #                (left turn), negative = clockwise (right turn).
        #
        # THRESHOLD: 0.1 rad/s (~5.7 °/s).
        #   Below this → the robot is considered to be moving straight.
        #   Above this → classified as turning left or right.
        #
        # WHY 0.1 rad/s?
        #   - Small enough that gentle curves (common on WRO tracks)
        #     are detected as turns.
        #   - Large enough that IMU noise / vibration doesn't cause
        #     false turn detections when driving straight.
        #
        # WHAT IF YOU CHANGE IT?
        #   ↑ (e.g. 0.2) → Only sharp turns are detected; the robot
        #     stays in "forward" on gentle curves.
        #   ↓ (e.g. 0.05) → Tiny wobbles trigger "left"/"right";
        #     the state machine may oscillate.

        if abs(heading_rate) < 0.1:
            self._direction = "forward"
        elif heading_rate > 0:
            self._direction = "left"
        else:
            self._direction = "right"

    @property
    def direction(self):
        # Read-only.  Returns "forward", "left", or "right".
        return self._direction

# ── What happens if you change key values? ─────────────────────────
# * The deadband 0.1 rad/s (~5.7 °/s) – ↓ makes the detector more
#   sensitive (tiny rotations are classified as turns);
#   ↑ makes it more tolerant (small wobbles ignored).
# * If the sign convention of heading_rate differs (e.g. positive =
#   right), swap the branches under elif/else.
# * This is a simple threshold; a real system might use a moving
#   average or hysteresis to avoid flickering.
# ────────────────────────────────────────────────────────────────────
