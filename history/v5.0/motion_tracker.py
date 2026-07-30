from dead_reckon import DeadReckon


class MotionTracker:
    def __init__(self):
        self.dr = DeadReckon()
        self.last_distance = 0.0
        self.segment_count = 0

    def log_segment(self, left_ticks: int, right_ticks: int,
                    real_distance: float) -> None:
        self.dr.update(left_ticks, right_ticks)
        est_distance = math.hypot(self.dr.x, self.dr.y)
        error = abs(est_distance - real_distance)
        self.segment_count += 1
        status = "OK" if error < 0.05 else "WARN" if error < 0.15 else "FAIL"
        print(
            f"[DEAD_RECKON] Segment {real_distance:.1f}m: "
            f"est={est_distance:.2f}m err={error*100:.0f}cm \u2190 {status}"
        )


if __name__ == "__main__":
    import math
    mt = MotionTracker()
    mt.log_segment(1600, 1600, 0.5)
    mt.log_segment(3200, 3200, 1.0)
    mt.log_segment(4800, 4800, 1.5)
    mt.log_segment(6400, 6400, 2.0)
