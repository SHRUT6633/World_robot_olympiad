from .adaptive_pid import AdaptivePID


class MotorPID(AdaptivePID):
    # PID controller specialised for speed control of a DC motor.
    # Converts a velocity error into a PWM duty cycle (0–255).
    #
    # The PID output is scaled by dt and added to the current speed command,
    # producing smooth acceleration/deceleration (velocity-form PID).

    def __init__(self, kp=0.5, ki=0.1, kd=0.01, dt=0.01):
        super().__init__(kp, ki, kd, dt)
        self.max_speed = 255   # Maximum PWM output (8-bit, 0–255)

    def compute_speed(self, target_v, current_v):
        # target_v: desired speed (m/s or scaled units)
        # current_v: current measured speed
        #
        # Returns a PWM duty cycle value clamped to [0, 255].
        error = target_v - current_v
        output = self.compute(error, limit=100)
        speed = current_v + output * self.dt
        return max(0, min(self.max_speed, speed))
