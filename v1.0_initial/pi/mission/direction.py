class DirectionDetector:
    def __init__(self):
        self._direction = "forward"

    def update(self, heading_rate):
        if abs(heading_rate) < 0.1:
            self._direction = "forward"
        elif heading_rate > 0:
            self._direction = "left"
        else:
            self._direction = "right"

    @property
    def direction(self):
        return self._direction
