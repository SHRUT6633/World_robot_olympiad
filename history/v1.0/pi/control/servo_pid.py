from .adaptive_pid import AdaptivePID


class ServoPID(AdaptivePID):
    def __init__(self, kp=0.8, ki=0.05, kd=0.02, dt=0.01):
        super().__init__(kp, ki, kd, dt)
        self.min_angle = -30.0
        self.max_angle = 30.0

    def compute_angle(self, target_steering, current_steering):
        error = target_steering - current_steering
        output = self.compute(error, limit=10.0)
        angle = current_steering + output * self.dt
        return max(self.min_angle, min(self.max_angle, angle))
