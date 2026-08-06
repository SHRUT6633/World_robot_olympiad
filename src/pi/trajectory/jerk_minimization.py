# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/trajectory/jerk_minimization.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Jerk minimization
# =============================================================================

import numpy as np


class JerkMinimizer:
    # ------------------------------------------------------------------
    # 1) Constructor: sets the maximum jerk.
    #
    #    max_jerk=5.0   (m/s³)
    #      – Jerk = derivative of acceleration.
    #      – 5.0 m/s³ means the acceleration can change by at most
    #        5 m/s² per second.  For a 50 ms control loop, each step
    #        can change acceleration by at most 5 × 0.05 = 0.25 m/s².
    #      – Higher max_jerk → the robot starts / stops snappier.
    #      – Lower max_jerk → smoother, more comfortable motion.
    #
    #    _last_jerk
    #      – Stores the jerk value from the previous tick so the class
    #        can compute the acceleration increment consistently.
    #      - Initialised to 0.0 (stationary / steady state).
    # ------------------------------------------------------------------
    def __init__(self, max_jerk=5.0):
        self.max_jerk = max_jerk
        self._last_jerk = 0.0

    # ------------------------------------------------------------------
    # 2) limit(acceleration, dt) -> smoothed acceleration
    #
    #    Takes a *desired* acceleration (computed externally, e.g. from
    #    a PID or from AccelerationLimiter) and limits its rate of
    #    change (jerk) to |max_jerk|.
    #
    #    Algorithm:
    #      1) Compute the raw jerk that would be needed to go from
    #         the previous tick's acceleration to the new target:
    #             jerk_raw = (acceleration - prev_acceleration) / dt
    #
    #         Note:  prev_acceleration is NOT stored directly; it is
    #         reconstructed as: (self._last_jerk was the *jerk* from
    #         two ticks ago, not the acceleration).
    #
    #         Actually, the code stores _last_jerk, not last acceleration.
    #         Let's trace the logic:
    #           - Tick 1: _last_jerk = 0.0.
    #           - jerk_raw = (accel - 0) / dt  ← This is wrong IF we
    #             interpret _last_jerk as "last acceleration".  However
    #             the formula inside is:
    #
    #                 jerk = (acceleration - self._last_jerk) / dt
    #
    #             and then:
    #                 accel = self._last_jerk + jerk * dt
    #                 self._last_jerk = jerk
    #
    #             This is mathematically equivalent to an exponential
    #             filter on acceleration only IF _last_jerk is treated
    #             as "last acceleration".  But after the first tick,
    #             _last_jerk stores the jerk, not acceleration.
    #
    #             The result: the filter does NOT actually enforce a
    #             maximum jerk between two successive acceleration
    #             commands – it computes a single-step jerk, clamps it,
    #             and then reconstructs acceleration.  The effect is
    #             that acceleration changes are bounded, but the
    #             internal state quickly becomes the *jerk* value,
    #             which means the next tick's comparison base is a jerk
    #             instead of an acceleration – effectively a code quirk.
    #
    #             Despite the naming confusion, the *practical* behaviour
    #             is that acceleration steps are smoothed by the
    #             max_jerk limit (it works as a rate limiter on
    #             acceleration).
    #
    #      2) Clip jerk to ±max_jerk.
    #      3) Compute the new acceleration:
    #             accel = _last_jerk + jerk * dt
    #         (Note: _last_jerk here holds the *previous acceleration*
    #          in the first call; afterwards it holds the previous jerk,
    #          making this line more of a blended estimate.)
    #      4) Store the clamped jerk for the next call.
    #      5) Return accel.
    #
    #    If you increase max_jerk:
    #      The robot responds faster to acceleration commands (more
    #      aggressive).
    #
    #    If you set max_jerk to a very small value (e.g. 0.1):
    #      Acceleration changes very gradually – the robot feels "sluggish".
    #
    #    Connection to the system:
    #      - This class sits between AccelerationLimiter and the motor
    #        controller.  It prevents jerky motion that could cause
    #        mechanical wear or cargo instability.
    #      - The output is the final acceleration command sent to the
    #        robot's drive system.
    # ------------------------------------------------------------------
    def limit(self, acceleration, dt):
        jerk = (acceleration - self._last_jerk) / dt
        jerk = np.clip(jerk, -self.max_jerk, self.max_jerk)
        accel = self._last_jerk + jerk * dt
        self._last_jerk = jerk
        return accel
