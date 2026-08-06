# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/motor_pid.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: PID controller specialised for DC motor speed control
# =============================================================================

from .adaptive_pid import AdaptivePID


class MotorPID(AdaptivePID):
    # PID controller specialised for speed control of a DC motor.
    # Converts a velocity error into a PWM duty cycle (0–255).
    #
    # The PID output is scaled by dt and added to the current speed command,
    # producing smooth acceleration/deceleration (velocity-form PID).

    # Default gains tuned for small DC gearmotor with encoder feedback at ~100 Hz loop rate
    # kp=0.5: moderate proportional response — avoids PWM saturation on step inputs
    # ki=0.1: higher integral for steady-state speed accuracy under varying load
    # kd=0.01: light derivative damping to suppress encoder quantisation noise
    def __init__(self, kp=0.5, ki=0.1, kd=0.01, dt=0.01):
        super().__init__(kp, ki, kd, dt)
        self.max_speed = 255   # Maximum PWM output (8-bit, 0–255)

    def compute_speed(self, target_v, current_v):
        # target_v: desired speed (m/s or scaled units)
        # current_v: current measured speed
        #
        # Returns a PWM duty cycle value clamped to [0, 255].
        # Velocity-form PID: compute delta from error, then integrate into current speed command
        # This avoids bumpy step changes — the motor accelerates/decelerates smoothly
        error = target_v - current_v
        # limit=100 clamps the PID delta to [-100, 100] to prevent aggressive jerk
        output = self.compute(error, limit=100)
        # Integrate the PID delta: speed += delta * dt → produces smooth ramping
        speed = current_v + output * self.dt
        return max(0, min(self.max_speed, speed))
