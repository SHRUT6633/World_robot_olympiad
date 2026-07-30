from .adaptive_pid import AdaptivePID


class MotorPID(AdaptivePID):
    def __init__(self, kp=0.5, ki=0.1, kd=0.01, dt=0.01):
        super().__init__(kp, ki, kd, dt)
        self.max_speed = 255

    def compute_speed(self, target_v, current_v):
        error = target_v - current_v
        output = self.compute(error, limit=100)
        speed = current_v + output * self.dt
        return max(0, min(self.max_speed, speed))
