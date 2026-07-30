class AdaptivePID:
    # Versatile PID (Proportional–Integral–Derivative) controller with optional output limiting.
    #
    # Control law:
    #   output = kp * error + ki * integral + kd * derivative
    #
    # where:
    #   integral   = sum(error * dt) over time
    #   derivative = (error - last_error) / dt
    #
    # kp (proportional gain): immediate reaction to current error.
    #   Higher kp = faster response but can cause overshoot/oscillation.
    #
    # ki (integral gain): eliminates steady-state error by accumulating past errors.
    #   Higher ki = faster zeroing of persistent error but can cause integral windup.
    #   Too high = instability (especially with slow response systems).
    #
    # kd (derivative gain): dampens oscillations by reacting to the error's rate of change.
    #   Higher kd = smoother response, less overshoot.
    #   Too high = amplifies measurement noise (jerky output).
    #
    # dt: time step between compute() calls (seconds). Must match the actual loop rate.
    #   If dt is wrong: integral and derivative terms will be mis-scaled.
    #
    # limit: optional symmetric output clamp [-limit, +limit].
    #   Prevents the controller from commanding excessive values.

    def __init__(self, kp=0.5, ki=0.05, kd=0.01, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self._integral = 0.0    # Accumulated integral of error over time
        self._last_error = 0.0  # Previous error for derivative calculation
        self._last_output = 0.0 # Most recent output value

    def compute(self, error, limit=None):
        # error = setpoint - measured_value (signed)
        # limit: if set, output is clamped to [-limit, +limit]
        self._integral += error * self.dt
        derivative = (error - self._last_error) / self.dt
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._last_error = error
        if limit is not None:
            output = max(-limit, min(limit, output))
        self._last_output = output
        return output

    def reset(self):
        # Reset internal integral and last_error to zero.
        # Call this when the setpoint changes significantly, or after a long pause,
        # to prevent integral windup and derivative spikes.
        self._integral = 0.0
        self._last_error = 0.0
