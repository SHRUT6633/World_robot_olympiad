# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/anti_windup.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Anti-windup to prevent integral accumulation during actuator saturation
# =============================================================================

class AntiWindup:
    # AntiWindup prevents the integral term in a PID controller from accumulating
    # excessively when the actuator is saturated (i.e., the output hits a physical
    # limit). This avoids "integral windup" — a large overshoot that happens when
    # the integral term keeps growing while the output is clamped.

    def __init__(self, clamp_min=-10, clamp_max=10):
        # clamp_min: lower bound of the allowable actuator output.
        # clamp_max: upper bound of the allowable actuator output.
        # These represent the physical limits of the motor/servo.
        self.clamp_min = clamp_min
        self.clamp_max = clamp_max

    # NOTE: Both methods are stubs that return `integral` unchanged.
    # In a real implementation, replace these with active clamping or back-calculation
    # logic to prevent the PID integral from growing while the actuator is saturated.
    def apply(self, integral, output):
        # Basic back-calculation or clamping anti-windup stub.
        # "integral": the current accumulated integral term.
        # "output": the total controller output (before clamping).
        # If the output exceeds the clamp bounds, the integral is frozen
        # (returned unchanged) so it cannot grow further.
        # NOTE: This implementation returns `integral` unchanged in all cases,
        # which effectively means anti-windup is a no-op at this level.
        # Changing clamp_min/clamp_max changes at what output levels the
        # integral would be frozen (if the logic were active).
        if output > self.clamp_max:
            return integral
        if output < self.clamp_min:
            return integral
        return integral

    @staticmethod
    def conditional(integral, error, output, limit):
        # Conditional integration (also called integrator clamping).
        # Freezes integration when the output is saturated AND the error
        # would cause the integral to grow in the same direction (making
        # windup worse).
        # - If output >= limit and error > 0: the controller is pushing
        #   against the limit and the error wants more — freeze integral.
        # - If output <= -limit and error < 0: same scenario in the
        #   negative direction.
        # Otherwise allow integration.
        # NOTE: This stub returns `integral` unchanged regardless.
        # Changing `limit` changes the saturation threshold.
        # Conditional integration: freeze integral if saturated AND error pushes same direction
        # This prevents windup more aggressively than simple output clamping
        if output >= limit and error > 0:
            return integral
        if output <= -limit and error < 0:
            return integral
        # Outside saturation — allow normal integration
        return integral
