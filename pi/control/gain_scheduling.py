# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/gain_scheduling.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Speed-based PID gain scheduling for varying dynamics
# =============================================================================

class GainScheduler:
    # Selects PID gains based on the robot's current speed (gain scheduling).
    # At different speeds, the robot's dynamics change significantly:
    #   - Slow: high gains for aggressive, precise control.
    #   - Medium: moderate gains.
    #   - Fast: lower gains to avoid oscillation / instability at speed.
    #
    # This is a simple if-else schedule. More advanced implementations could
    # interpolate between gain sets for smooth transitions.

    def __init__(self):
        # _gains: a dictionary mapping speed zones to PID gain dictionaries.
        # Each zone has kp (proportional), ki (integral), kd (derivative).
        # As speed increases, all gains decrease — this prevents instability
        # (high-speed oscillations) at the cost of slower error correction.
        #
        # Changing these values affects closed-loop behavior directly:
        #   Higher kp/ki/kd in a zone = more aggressive but potentially unstable.
        #   Lower values = more stable but slower to correct errors.
        self._gains = {
            "slow": {"kp": 0.8, "ki": 0.05, "kd": 0.02},
            "medium": {"kp": 0.5, "ki": 0.03, "kd": 0.01},
            "fast": {"kp": 0.3, "ki": 0.01, "kd": 0.005},
        }
        # current_zone: tracks which zone was last selected (for debugging/logging).
        self.current_zone = "medium"

    def select(self, v):
        # v: current speed of the robot (m/s).
        # Returns: the gain dict for the corresponding speed zone.
        #
        # Speed thresholds (0.5, 1.5 m/s) define zone boundaries.
        # Changing these shifts when gains transition:
        #   - Lower thresholds = earlier reduction of gains (more conservative).
        #   - Higher thresholds = aggressive gains held to higher speed (riskier).
        # Speed zones chosen based on WRO arena characteristics:
        #   < 0.5 m/s: walking speed, tight turns require aggressive gains
        #   0.5–1.5 m/s: cruising speed, moderate gains balance tracking vs stability
        #   > 1.5 m/s: high-speed runs, reduced gains prevent oscillatory instability
        # No hysteresis on zone boundaries — gains can chatter at transition speeds
        if v < 0.5:
            self.current_zone = "slow"
        elif v < 1.5:
            self.current_zone = "medium"
        else:
            self.current_zone = "fast"
        return self._gains[self.current_zone]
