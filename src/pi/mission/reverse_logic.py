# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/reverse_logic.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Reverse manoeuvre logic
# =============================================================================

class ReverseLogic:
    # ── TIMED REVERSE MANOEUVRE MANAGER ──────────────────────────────
    #
    # Decides when the robot should back up and manages the duration of
    # the reverse manoeuvre.  This is a simple timed state: once started,
    # it runs for at least min_time seconds, then signals that the robot
    # should stop reversing and try forward motion again.
    #
    # WHY a dedicated module?
    #   • Prevents the robot from oscillating (forward/reverse/forward)
    #     by enforcing a minimum reverse duration.
    #   • Keeps the "when to reverse" logic separate from "how long to
    #     reverse".
    #   • The state machine uses this to transition to/from REVERSE state.
    #
    # STATE MACHINE INTEGRATION:
    #   Transition INTO REVERSE:
    #     The mission planner calls should_reverse() every tick while in
    #     FORWARD or OBSTACLE_AVOID.  If it returns True, it transitions
    #     to REVERSE and calls start_reverse().
    #
    #   Transition OUT OF REVERSE:
    #     The mission planner calls update(dt) every tick while in REVERSE.
    #     When update() returns False (minimum time elapsed), it transitions
    #     back to OBSTACLE_AVOID (to try a different direction).
    #
    # ERROR RECOVERY:
    #   If the robot keeps reversing into the same dead-end (e.g. it backs
    #   up, turns the same way, and gets stuck again), the race strategy
    #   can flip the ObstacleStrategy preference after each reverse cycle.
    #   This breaks the symmetry and eventually finds an escape.

    def __init__(self, min_reverse_time=0.5, max_reverse_time=2.0):
        # ── Timing parameters ──
        # min_reverse_time (seconds):
        #   Minimum time the robot must reverse before it can try forward
        #   again.  WHY?  If the robot reverses for only 0.1 s and then
        #   tries forward, it may immediately re-detect the obstacle and
        #   oscillate.  0.5 s gives enough distance to clear the obstacle
        #   or approach it from a different angle.
        #
        # max_reverse_time (seconds):
        #   Maximum time the robot should reverse before forcing a
        #   different action (e.g. a sharper turn).  Currently stored
        #   but NOT enforced in update() — it's available for the
        #   mission planner to use externally if needed.
        self.min_time = min_reverse_time
        self.max_time = max_reverse_time

        self._reversing = False   # True while a reverse manoeuvre is active.
        self._elapsed = 0.0       # Elapsed time of the current reverse.

    def should_reverse(self, blocked_front, blocked_left, blocked_right):
        # ── DECISION RULE ──
        # Returns True if the robot is boxed in and must reverse.
        #
        # LOGIC:
        #   Reverse if the front is blocked AND at least one side is
        #   also blocked.  WHY?  If the front is blocked but a side is
        #   free, the robot can simply steer away.  Reversing is only
        #   needed when there's no escape direction.
        #
        # This avoids unnecessary reversing (which costs time and may
        # confuse the localisation system).
        return blocked_front and (blocked_left or blocked_right)

    def update(self, dt):
        # ── TIMED REVERSE UPDATE ──
        # Called every tick while the robot is in the REVERSE state.
        #
        # Returns:
        #   True  → keep reversing (timer not yet expired).
        #   False → minimum time elapsed, stop reversing.
        #
        # WHY a timer instead of distance?
        #   Distance estimation during reversing is unreliable (wheel
        #   slip, uneven surface).  A time-based approach is simpler
        #   and "good enough" for WRO obstacle configurations.
        if self._reversing:
            self._elapsed += dt
            if self._elapsed >= self.min_time:
                # Timer expired: stop reversing.
                self._reversing = False
                self._elapsed = 0.0
                return False
            return True
        return False

    def start_reverse(self):
        # Begin a reverse manoeuvre.
        # Resets the elapsed timer to 0 so the min_time count starts
        # fresh.  Called by the mission planner when transitioning
        # into the REVERSE state.
        self._reversing = True
        self._elapsed = 0.0
