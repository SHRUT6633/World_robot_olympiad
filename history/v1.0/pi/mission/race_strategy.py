from ..system.logger import log


class RaceStrategy:
    def __init__(self):
        self.mode = "normal"

    def update(self, lap, elapsed, obstacles):
        if lap == 0:
            self.mode = "exploration"
        elif lap >= 1:
            self.mode = "racing"
        if obstacles > 3:
            self.mode = "cautious"

    @property
    def speed_factor(self):
        factors = {"exploration": 0.6, "normal": 0.8, "racing": 1.0, "cautious": 0.5}
        return factors.get(self.mode, 0.7)
