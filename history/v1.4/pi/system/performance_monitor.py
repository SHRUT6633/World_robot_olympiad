# =============================================================================
# performance_monitor.py — PerformanceMonitor (CPU/RAM Background Logger)
# =============================================================================
# Monitors system resource usage (CPU percent, RAM percent) on a daemon
# background thread every interval_s seconds. Metrics are stored in a
# ring buffer (max 600 samples = 10 minutes at 1 Hz) and can be queried
# for rolling averages or a report dict.
#
# Why a separate thread instead of an async task?
#   - psutil calls may block briefly (e.g., cpu_percent(interval=0.1)).
#   - Running them in the async event loop would stall all tasks.
#   - A daemon thread is the standard approach for non-async monitoring.
#
# Key parameters:
#   interval_s – Sampling interval in seconds (default 1.0).
#                At 1 Hz, 600 samples = 10 minutes of history.
#                If the robot runs longer than 10 minutes, old samples
#                are dropped (ring buffer).
#
# Called by:
#   - SystemManager.init_all()  →  perf.start()
#   - SystemManager.stop()      →  perf.stop()
#   - SystemManager.shutdown()  →  log perf.report()
#   - diagnostics.py            →  snapshot includes perf.report()
#
# What happens if you change interval_s?
#   interval_s = 0.5 → samples at 2 Hz, 600 samples = 5 min history.
#   interval_s = 2.0 → samples at 0.5 Hz, 600 samples = 20 min history.
#   Lower interval = more precise CPU tracking but slightly more overhead.
#
# Metrics:
#   cpu – CPU usage percent (0-100). Collected via psutil.cpu_percent().
#           The interval=0.1 parameter means psutil measures CPU over a
#           100 ms window. This is fast and non-blocking in practice.
#   ram – RAM usage percent (0-100). Collected via psutil.virtual_memory().
#           This is an instantaneous read (no blocking).
#   temp – Temperature (placeholder). Currently not populated; psutil does
#           not expose Pi CPU temperature. On Raspberry Pi, you would use
#           vcgencmd measure_temp or read /sys/class/thermal/... This slot
#           exists for future expansion.
# =============================================================================

import time
import psutil
import threading
from .logger import log


class PerformanceMonitor:
    def __init__(self, interval_s=1.0):
        # Sampling interval in seconds. 1.0 = one sample per second.
        self.interval_s = interval_s
        # Flag controlling the background thread loop.
        self._running = False
        # Background thread reference (daemon=True so it dies with main).
        self._thread = None
        # Metrics ring buffer: each key maps to a list of float values.
        # cpu: CPU utilization percent (0–100)
        # ram: RAM utilization percent (0–100)
        # temp: CPU temperature (placeholder, always empty unless implemented)
        self._metrics = {"cpu": [], "ram": [], "temp": []}

    # -------------------------------------------------------------------------
    # start()
    # -------------------------------------------------------------------------
    # Starts the background monitoring daemon thread.
    # Called by: SystemManager.init_all() after all components are init'd.
    # The thread calls _monitor() in a loop until stop() is called.
    # -------------------------------------------------------------------------
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    # -------------------------------------------------------------------------
    # stop()
    # -------------------------------------------------------------------------
    # Sets the running flag to False. The _monitor loop will exit on the
    # next iteration. The thread may still be alive briefly while finishing
    # its current sleep().
    # Called by: SystemManager.stop().
    # -------------------------------------------------------------------------
    def stop(self):
        self._running = False

    # -------------------------------------------------------------------------
    # _monitor()
    # -------------------------------------------------------------------------
    # Thread target: loops while _running is True, sampling CPU and RAM.
    #   - psutil.cpu_percent(interval=0.1): measures CPU utilization over a
    #     100 ms window. Returns a float (0-100).
    #   - psutil.virtual_memory().percent: returns current RAM usage %.
    #   - Appends each sample to the ring buffer.
    #   - If the buffer exceeds 600 samples, trims from the front.
    #     (600 samples at 1 Hz = 10 minutes of history.)
    #   - Catches all exceptions and logs a warning (never crashes).
    #   - Sleeps for interval_s seconds between samples.
    # -------------------------------------------------------------------------
    def _monitor(self):
        while self._running:
            try:
                # cpu_percent(interval=0.1) blocks for ~100 ms to measure CPU.
                # The result is the average utilization over that window.
                cpu = psutil.cpu_percent(interval=0.1)
                # virtual_memory().percent is a snapshot of current usage.
                ram = psutil.virtual_memory().percent
                self._metrics["cpu"].append(cpu)
                self._metrics["ram"].append(ram)
                # Trim ring buffer to keep max 600 samples.
                if len(self._metrics["cpu"]) > 600:
                    for k in self._metrics:
                        self._metrics[k] = self._metrics[k][-600:]
            except Exception as e:
                log.warn(f"PerfMonitor: {e}")
            time.sleep(self.interval_s)

    # -------------------------------------------------------------------------
    # cpu_avg (property)
    # -------------------------------------------------------------------------
    # Returns the average CPU% over the last 10 samples (rolling window).
    # If fewer than 10 samples exist, averages whatever is available.
    # -------------------------------------------------------------------------
    @property
    def cpu_avg(self):
        return round(sum(self._metrics["cpu"][-10:]) / max(len(self._metrics["cpu"][-10:]), 1), 1)

    # -------------------------------------------------------------------------
    # ram_avg (property)
    # -------------------------------------------------------------------------
    # Returns the average RAM% over the last 10 samples (rolling window).
    # -------------------------------------------------------------------------
    @property
    def ram_avg(self):
        return round(sum(self._metrics["ram"][-10:]) / max(len(self._metrics["ram"][-10:]), 1), 1)

    # -------------------------------------------------------------------------
    # report() -> dict
    # -------------------------------------------------------------------------
    # Returns a compact summary dict with current averages.
    # Called by:
    #   - SystemManager.shutdown() for final log output.
    #   - diagnostics.py snapshot() to include in diagnostic dumps.
    # -------------------------------------------------------------------------
    def report(self):
        return {"cpu_avg": self.cpu_avg, "ram_avg": self.ram_avg}
