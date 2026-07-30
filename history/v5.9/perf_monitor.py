import time


class PerfMonitor:
    def __init__(self, window: int = 100):
        self.window = window
        self._times = {"predict": [], "correct": [], "verify": []}

    def record(self, step: str, elapsed: float) -> None:
        self._times[step].append(elapsed)
        if len(self._times[step]) > self.window:
            self._times[step].pop(0)

    def avg_ms(self, step: str) -> float:
        vals = self._times.get(step, [])
        return (sum(vals) / len(vals) * 1000) if vals else 0.0

    def report(self) -> str:
        parts = []
        for step in ["predict", "correct", "verify"]:
            avg = self.avg_ms(step)
            hz = 1000.0 / max(avg, 0.001)
            parts.append(f"{step}={avg:.1f}ms ({hz:.0f}Hz)")
        return " | ".join(parts)


if __name__ == "__main__":
    pm = PerfMonitor()
    import random
    for _ in range(200):
        pm.record("predict", 0.0005 + random.random() * 0.0005)
        pm.record("correct", 0.001 + random.random() * 0.002)
        pm.record("verify", 0.003 + random.random() * 0.004)
    print(f"[PERF] {pm.report()}")
