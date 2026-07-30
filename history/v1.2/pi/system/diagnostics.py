import time
from .logger import log

class Diagnostics:
    def __init__(self, manager=None):
        self.manager = manager
        self._history = []

    def snapshot(self):
        snap = {
            "timestamp": time.perf_counter(),
            "tasks": {},
            "health": {},
        }
        if self.manager:
            snap["tasks"] = self.manager.scheduler.stats()
            snap["health"] = self.manager.health.status
            snap["perf"] = self.manager.perf.report()
        self._history.append(snap)
        if len(self._history) > 1000:
            self._history.pop(0)
        return snap

    def dump(self):
        import json
        return json.dumps(self._history[-100:], indent=2)
