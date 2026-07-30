# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/obstacle_strategy.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Obstacle avoidance strategy
# =============================================================================

from ..system.logger import log


class ObstacleStrategy:
    # ── REACTIVE OBSTACLE AVOIDANCE ──────────────────────────────────
    #
    # Decides which way to steer when the robot faces an obstacle.
    # This is a purely reactive (non-planning) layer: it uses three
    # boolean flags from the obstacle detection system and returns a
    # steering direction.
    #
    # WHY reactive instead of path-planning?
    #   - Obstacles in WRO are typically simple (cones, blocks).  A
    #     reactive bumper-style approach is fast and sufficient.
    #   - Global re-planning (e.g. A*) would be overkill and slower.
    #   - The ObstacleStrategy runs in the control loop (every tick)
    #     while the robot is in the OBSTACLE_AVOID or FORWARD state.
    #
    # SENSOR MODEL:
    #   Three virtual "walls" at left, front, right of the robot.
    #   Each is True if an obstacle is detected within a threshold
    #   distance.  The thresholds are set by the sensor driver
    #   (ultrasonic / IR / ToF) and passed in by the mission planner.
    #
    # DECISION MATRIX:
    #
    #   front  left  right  →  action       rationale
    #   ─────────────────────────────────────────────────
    #   F      x     x         forward      No blocking obstacle.
    #   T      T     F         right        Only right is free.
    #   T      F     T         left         Only left is free.
    #   T      F     F         <prefer>     Both free → use preference.
    #   T      T     T         reverse      Trapped → back up.
    #
    # STATE MACHINE INTEGRATION:
    #   The mission planner calls decide() and uses the result to
    #   command the motor controller.  It also uses the result to
    #   trigger state transitions:
    #     "forward" → stay in FORWARD (or transition FROM OBSTACLE_AVOID).
    #     "left"/"right" → transition to OBSTACLE_AVOID.
    #     "reverse" → transition to REVERSE.
    #
    # ERROR RECOVERY:
    #   If the robot gets stuck oscillating (left/right/left/right),
    #   the mission planner can call set_preference() to flip the bias,
    #   breaking the cycle.

    # Class-level constants so callers don't hard-code strings.
    LEFT = "left"
    RIGHT = "right"

    def __init__(self, prefer=LEFT):
        # prefer – default turning bias when both left and right are open.
        #
        # WHY have a preference?
        #   When both sides are free (e.g. obstacle is small and centred),
        #   the robot must pick one.  A fixed preference avoids stochastic
        #   behaviour.  The WRO track layout determines the preferred side:
        #   e.g. left-wall following is common when the track has a
        #   continuous left boundary.
        #
        #   If the robot gets stuck (e.g. in a dead-end), the race
        #   strategy can flip this at runtime via set_preference().
        self.prefer = prefer

    def decide(self, wall_left, wall_right, wall_front):
        # ── MAIN DECISION LOGIC ──
        # Returns one of: "forward", "left", "right", "reverse".
        #
        # PARAMETERS:
        #   wall_left  – True if obstacle detected on the left side.
        #   wall_right – True if obstacle detected on the right side.
        #   wall_front – True if obstacle detected directly ahead.
        #
        # All three are boolean: the sensor fusion layer handles
        # distance thresholds and noise filtering before calling this.
        #
        # WHY three booleans instead of distance values?
        #   Simplicity and speed.  The trajectory planner (which would
        #   need exact distances) is not used here.

        if wall_front:
            # ── Obstacle ahead: need to steer around it ──
            # Determine which sides are free for escape.
            free_left = not wall_left if not wall_left else False
            # NOTE: This line is functionally equivalent to `free_left = not wall_left`.
            # The `else False` branch is dead code (only reached when
            # `not wall_left` is False, i.e. wall_left is True, which
            # also produces False).  Left as-is for historical reasons.
            free_right = not wall_right if not wall_right else False

            if free_left and free_right:
                # ── Both sides open: use preference ──
                # The obstacle is small and centred.  Use the default
                # bias to turn consistently.
                return self.prefer
            elif free_left:
                # ── Only left is free: turn left ──
                return self.LEFT
            elif free_right:
                # ── Only right is free: turn right ──
                return self.RIGHT
            else:
                # ── Trapped: all three directions blocked ──
                # Must reverse to get out of the boxed-in situation.
                # The reverse_logic module handles the timed backup.
                return "reverse"

        # ── No frontal obstacle: continue straight ──
        # Even if sides are blocked, if the front is clear, the robot
        # can keep moving forward.  Side obstacles will be handled by
        # wall-following if active.
        return "forward"

    def set_preference(self, pref):
        # Dynamically change the turning preference.
        #
        # WHY at runtime?
        #   - After reversing, the robot may want to try the opposite
        #     direction to avoid re-entering the same dead-end.
        #   - The race strategy can adapt the preference based on the
        #     lap number or track section (e.g. prefer left on lap 1,
        #     right on lap 2 after learning the layout).
        self.prefer = pref

# ── What happens if you change key values? ─────────────────────────
# * The assignment `free_left = not wall_left if not wall_left else False`
#   is a convoluted way of writing `free_left = not wall_left`.
#   The `if not wall_left else False` branch is never reached because
#   when `wall_left` is True, the `if not wall_left` condition is
#   False, so it goes to `else False` which also evaluates to False.
#   Simplifying to `free_left = not wall_left` is clearer and has no
#   behavioural difference.
# * Changing the preference changes the robot's "handedness"
#   (left-wall follower vs right-wall follower).
# * If you add more states (e.g. "slight_left", "hard_right") the
#   logic would need richer sensor inputs (e.g. distances, not just
#   booleans).
# ────────────────────────────────────────────────────────────────────
