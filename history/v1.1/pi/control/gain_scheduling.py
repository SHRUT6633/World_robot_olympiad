class GainScheduler:
    def __init__(self):
        self._gains = {
            "slow": {"kp": 0.8, "ki": 0.05, "kd": 0.02},
            "medium": {"kp": 0.5, "ki": 0.03, "kd": 0.01},
            "fast": {"kp": 0.3, "ki": 0.01, "kd": 0.005},
        }
        self.current_zone = "medium"

    def select(self, v):
        if v < 0.5:
            self.current_zone = "slow"
        elif v < 1.5:
            self.current_zone = "medium"
        else:
            self.current_zone = "fast"
        return self._gains[self.current_zone]
