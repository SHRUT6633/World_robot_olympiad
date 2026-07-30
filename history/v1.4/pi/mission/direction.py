class DirectionDetector:
    # ──────────────────────────────────────────────────────────────────
    # Infers the robot's *turning direction* from the instantaneous
    # heading rate (angular velocity).
    #
    # Used by higher-level mission logic to decide behaviour:
    #   - "forward" – going straight → continue.
    #   - "left" / "right" – turning → may trigger a state change.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        # Internal state; default direction is forward.
        self._direction = "forward"

    def update(self, heading_rate):
        # heading_rate – angular velocity (rad / s).
        #   |rate| < 0.1  → essentially straight.
        #   rate > 0      → turning left    (positive = CCW).
        #   rate < 0      → turning right   (negative = CW).

        if abs(heading_rate) < 0.1:
            self._direction = "forward"
        elif heading_rate > 0:
            self._direction = "left"
        else:
            self._direction = "right"

    @property
    def direction(self):
        # Read-only access to the current direction string.
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
