import numpy as np


class AccelerationLimiter:
    # ------------------------------------------------------------------
    # 1) Constructor: sets the positive and negative acceleration limits.
    #
    #    max_accel=2.0   (m/s²)
    #      – The maximum *increase* in speed per second the controller
    #        will allow.  2.0 m/s² is a gentle acceleration (≈0.2 g).
    #
    #    max_decel=3.0   (m/s²)
    #      – The maximum *decrease* in speed per second (braking).
    #      – Typically larger than accel because braking is often
    #        stronger (e.g. emergency stops).  3.0 m/s² ≈ 0.3 g.
    #
    #    Changing these:
    #      - Larger values → more aggressive (snappier) motion.
    #      - Smaller values → gentler, safer, but may fail to stop
    #        in time during obstacle avoidance.
    # ------------------------------------------------------------------
    def __init__(self, max_accel=2.0, max_decel=3.0):
        self.max_a = max_accel
        self.max_d = max_decel

    # ------------------------------------------------------------------
    # 2) limit(current_v, target_v, dt) -> new velocity
    #
    #    Given the current speed (current_v), the desired speed
    #    (target_v), and the time step (dt), return a *feasible*
    #    velocity that respects the acceleration / deceleration limits.
    #
    #    Algorithm:
    #      dv = target_v - current_v    (desired velocity change)
    #
    #      If dv > 0 (speeding up):
    #        Clamp dv to at most max_a * dt
    #      If dv < 0 (slowing down):
        #        Clamp dv to at least -max_d * dt  (i.e. do not exceed
    #        deceleration limit)
    #
    #      Return current_v + clamped_dv
    #
    #    Mathematical rationale:
    #      v_new = current_v + clamp(dv, -max_d * dt, max_a * dt)
    #
    #    Edge case:
    #      - If dt is very small (e.g. 0.001 s), the allowable delta is
    #        tiny, so the velocity changes very gradually – this is
    #        physically correct.
    #      - If dt is unexpectedly large, the clamp still prevents
    #        instantaneous jumps.
    #
    #    Connection to the system:
    #      - Called every control loop iteration (e.g. 20 Hz / 50 ms)
    #        with the output of JerkMinimizer (or directly with the
    #        desired cruise speed).
    #      - The result becomes the new speed command to the motor driver.
    # ------------------------------------------------------------------
    def limit(self, current_v, target_v, dt):
        dv = target_v - current_v
        if dv > 0:
            dv = min(dv, self.max_a * dt)
        else:
            dv = max(dv, -self.max_d * dt)
        return current_v + dv
