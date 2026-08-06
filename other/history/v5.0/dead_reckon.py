import math
from collections import deque

WHEEL_BASE = 0.16       # meters
TICKS_PER_METER = 3200  # from motor calibration
MAX_DEAD_RECKON_DIST = 1.0  # meters


class DeadReckon:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.cumulative_distance = 0.0
        self._tick_buffer = deque(maxlen=5)

    def update(self, left_ticks: int, right_ticks: int) -> None:
        d_left = (left_ticks - self.left_ticks) / TICKS_PER_METER
        d_right = (right_ticks - self.right_ticks) / TICKS_PER_METER
        self.left_ticks = left_ticks
        self.right_ticks = right_ticks

        if abs(d_left - d_right) < 1e-9:
            d_forward = d_left
            d_heading = 0.0
        else:
            d_forward = (d_left + d_right) / 2.0
            d_heading = (d_right - d_left) / WHEEL_BASE

        self.heading = (self.heading + d_heading) % (2 * math.pi)
        self.x += d_forward * math.cos(self.heading)
        self.y += d_forward * math.sin(self.heading)
        self.cumulative_distance += abs(d_forward)

        self._tick_buffer.append((d_left, d_right))
        if self.cumulative_distance > MAX_DEAD_RECKON_DIST:
            import warnings
            warnings.warn(
                f"Cumulative path {self.cumulative_distance:.2f}m exceeds "
                f"{MAX_DEAD_RECKON_DIST}m threshold. Position uncertainty: HIGH"
            )

    def reset(self, x=0.0, y=0.0, heading=0.0) -> None:
        self.x = x
        self.y = y
        self.heading = heading
        self.cumulative_distance = 0.0
        self._tick_buffer.clear()

    def pose(self) -> tuple:
        return self.x, self.y, self.heading
