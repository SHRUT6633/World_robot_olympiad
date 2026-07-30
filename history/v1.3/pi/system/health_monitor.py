import time
from collections import defaultdict
from .logger import log

class HealthMonitor:
    def __init__(self, timeout_s=2.0):
        self.timeout_s = timeout_s
        self._last_heartbeat = {}
        self._status = {}

    def heartbeat(self, component: str):
        self._last_heartbeat[component] = time.perf_counter()
        self._status[component] = "alive"

    def check(self, component: str) -> bool:
        last = self._last_heartbeat.get(component, 0)
        dt = time.perf_counter() - last
        if dt > self.timeout_s:
            self._status[component] = "dead"
            return False
        self._status[component] = "alive"
        return True

    def check_all(self):
        results = {}
        for comp in self._last_heartbeat:
            results[comp] = self.check(comp)
        return results

    @property
    def status(self):
        return dict(self._status)

    def summary(self):
        alive = sum(1 for s in self._status.values() if s == "alive")
        dead = sum(1 for s in self._status.values() if s == "dead")
        return f"{alive}/{len(self._status)} components alive"
