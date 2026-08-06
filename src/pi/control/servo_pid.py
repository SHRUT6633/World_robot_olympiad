# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/servo_pid.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: PID controller specialised for servo-based steering
# =============================================================================

from .adaptive_pid import AdaptivePID


class ServoPID(AdaptivePID):
    # PID controller specialised for servo-based steering.
    # Converts a target steering angle error into a servo angle command.
    #
    # The servo angle is limited to [-30, 30] degrees (mechanical limits of the servo).
    # The PID output is scaled by dt and added to the current angle to produce
    # a smooth, incremental change (velocity-form PID on the steering angle).

    # Higher kp than MotorPID for quick steering step response; lower ki to avoid
    # oscillation from servo dead-zone and non-linearity
    # kd=0.02 provides moderate damping — critical for overshoot on sharp corners
    def __init__(self, kp=0.8, ki=0.05, kd=0.02, dt=0.01):
        super().__init__(kp, ki, kd, dt)
        self.min_angle = -30.0   # Minimum servo angle (degrees)
        self.max_angle = 30.0    # Maximum servo angle (degrees)

    def compute_angle(self, target_steering, current_steering):
        # target_steering: desired steering angle from path controller (degrees)
        # current_steering: current servo angle feedback (degrees)
        #
        # Returns a new servo angle clamped to [-30, 30] degrees.
        # Steering error computed in angle-space (degrees), not actuator command-space
        error = target_steering - current_steering
        # limit=10.0 ensures max delta of 10°/step — prevents servo jitter from large step inputs
        output = self.compute(error, limit=10.0)
        # Incremental (velocity-form) update: integrate PID output into servo angle
        angle = current_steering + output * self.dt
        return max(self.min_angle, min(self.max_angle, angle))
