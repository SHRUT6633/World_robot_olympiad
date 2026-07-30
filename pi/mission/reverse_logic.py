# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/reverse_logic.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Reverse manoeuvre logic
# =============================================================================

class ReverseLogic:
    # ReverseLogic decides when the robot should back up and manages the
    # duration of the reverse manoeuvre.  It is a simple timed state:
    # once started, it runs for at least min_time seconds, then signals
    # that the robot should stop reversing and try forward motion again.
    # The state machine uses this to transition to/from the REVERSE state.

    def __init__(self, min_reverse_time=0.5, max_reverse_time=2.0):
        # min_reverse_time -- minimum seconds to reverse before re-evaluating.
        #   Prevents the robot from twitching forward/backward too quickly.
        # max_reverse_time -- maximum seconds to reverse before forcing
        #   a different action (e.g. a sharper turn).  Currently not enforced
        #   in update() but available for external use.
        self.min_time = min_reverse_time
        self.max_time = max_reverse_time
        self._reversing = False   # Whether a reverse manoeuvre is active.
        self._elapsed = 0.0       # Time already spent reversing.

    def should_reverse(self, blocked_front, blocked_left, blocked_right):
        # Decision rule: reverse if the front is blocked AND at least one
        # side is also blocked (the robot is boxed in and cannot simply
        # steer away).  This avoids reversing unnecessarily when the front
        # is blocked but a side is open (the robot can just turn).
        return blocked_front and (blocked_left or blocked_right)

    def update(self, dt):
        # Called every tick while reversing.
        # Returns True if the robot should continue reversing,
        # False if the minimum reverse time has elapsed and the
        # robot should stop.
        if self._reversing:
            self._elapsed += dt
            if self._elapsed >= self.min_time:
                self._reversing = False
                self._elapsed = 0.0
                return False
            return True
        return False

    def start_reverse(self):
        # Begin a reverse manoeuvre and reset the timer.
        self._reversing = True
        self._elapsed = 0.0
