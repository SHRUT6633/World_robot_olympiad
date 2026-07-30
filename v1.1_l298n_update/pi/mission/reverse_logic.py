class ReverseLogic:
    def __init__(self, min_reverse_time=0.5, max_reverse_time=2.0):
        self.min_time = min_reverse_time
        self.max_time = max_reverse_time
        self._reversing = False
        self._elapsed = 0.0

    def should_reverse(self, blocked_front, blocked_left, blocked_right):
        return blocked_front and (blocked_left or blocked_right)

    def update(self, dt):
        if self._reversing:
            self._elapsed += dt
            if self._elapsed >= self.min_time:
                self._reversing = False
                self._elapsed = 0.0
                return False
            return True
        return False

    def start_reverse(self):
        self._reversing = True
        self._elapsed = 0.0
