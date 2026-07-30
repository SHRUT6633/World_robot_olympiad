from ..system.logger import log


class ObstacleStrategy:
    # ──────────────────────────────────────────────────────────────────
    # Decides which way to steer when the robot faces an obstacle.
    #
    # Uses three boolean sensor flags: wall_left, wall_right, wall_front.
    # The "preference" (left / right) breaks symmetry when both sides
    # are free.
    # ──────────────────────────────────────────────────────────────────

    # Class-level constants for readability.
    LEFT = "left"
    RIGHT = "right"

    def __init__(self, prefer=LEFT):
        # prefer – default turning preference when both sides are open.
        self.prefer = prefer

    def decide(self, wall_left, wall_right, wall_front):
        # Returns a string action:
        #   "forward" – no obstacle ahead, keep going.
        #   "left"    – turn left.
        #   "right"   – turn right.
        #   "reverse" – all directions blocked → back up.

        if wall_front:
            # Obstacle directly ahead.
            # Determine which sides are free.
            free_left = not wall_left if not wall_left else False   # equivalent to: free_left = not wall_left
            free_right = not wall_right if not wall_right else False

            if free_left and free_right:
                # Both sides open – use the configured preference.
                return self.prefer
            elif free_left:
                return self.LEFT
            elif free_right:
                return self.RIGHT
            else:
                # Trapped – no free direction.
                return "reverse"

        # No frontal obstacle → continue straight.
        return "forward"

    def set_preference(self, pref):
        # Update the turning preference at runtime.
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
