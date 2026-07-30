import time
import psutil
import threading
from .logger import log

class PerformanceMonitor:
    def __init__(self, interval_s=1.0):
        self.interval_s = interval_s
        self._running = False
        self._thread = None
        self._metrics = {"cpu": [], "ram": [], "temp": []}

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _monitor(self):
        while self._running:
            try:
                cpu = psutil.cpu_percent(interval=0.1)
                ram = psutil.virtual_memory().percent
                self._metrics["cpu"].append(cpu)
                self._metrics["ram"].append(ram)
                if len(self._metrics["cpu"]) > 600:
                    for k in self._metrics:
                        self._metrics[k] = self._metrics[k][-600:]
            except Exception as e:
                log.warn(f"PerfMonitor: {e}")
            time.sleep(self.interval_s)

    @property
    def cpu_avg(self):
        return round(sum(self._metrics["cpu"][-10:]) / max(len(self._metrics["cpu"][-10:]), 1), 1)

    @property
    def ram_avg(self):
        return round(sum(self._metrics["ram"][-10:]) / max(len(self._metrics["ram"][-10:]), 1), 1)

    def report(self):
        return {"cpu_avg": self.cpu_avg, "ram_avg": self.ram_avg}
