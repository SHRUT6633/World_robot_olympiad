# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/system/diagnostics.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# System-state snapshot and diagnostic dump
# =============================================================================

# =============================================================================
# diagnostics.py — Diagnostics (System-State Snapshot & Dump)
# =============================================================================
# Provides a lightweight diagnostic tool that periodically captures the
# state of the running system (task scheduler stats, health status,
# performance metrics) and stores them in an in-memory ring buffer.
#
# This is useful for:
#   - Post-run analysis: dump the last 100 snapshots to a JSON log.
#   - Real-time debugging: inspect the latest snapshot on demand.
#   - Detecting trends: e.g., CPU creeping up over time.
#
# Design:
#   - snapshot() gathers current state from the SystemManager (scheduler,
#     health monitor, performance monitor) and appends it to _history.
#   - _history holds up to 1000 snapshots. If the robot runs for a long
#     time and snapshots are taken frequently, old ones are discarded.
#   - dump() returns the most recent 100 snapshots as a JSON string.
#
# Connection to other files:
#   - SystemManager is passed via constructor (manager=...) or can be set
#     later. The Diagnostics object reads:
#       .scheduler.stats()   → Task timing stats (scheduler.py)
#       .health.status       → Alive/dead component status (health_monitor.py)
#       .perf.report()       → CPU/RAM averages (performance_monitor.py)
#   - There is currently NO code in main.py that instantiates or calls
#     Diagnostics — it is available for integration but not wired into
#     the main loop. To use it, add something like:
#       diag = Diagnostics(manager=mgr)
#       # call diag.snapshot() periodically in a low-Hz task
#       # call diag.dump() on shutdown or demand
#
# What changes would wiring it in require?
#   - In main.py, create Diagnostics(mgr) and add a diagnostics_task at
#     ~1-10 Hz that calls snapshot().
#   - The _history ring buffer (1000 entries) would grow to ~1 MB at
#     ~1 KB per snapshot. At 10 Hz, 1000 entries = 100 seconds.
#     Adjust the max size as needed.
# =============================================================================

import time
from .logger import log


class Diagnostics:
    def __init__(self, manager=None):
        # Reference to SystemManager (optional). If provided, snapshot()
        # gathers data from the manager's scheduler, health, and perf
        # sub-objects. If None, snapshot() returns timestamps only.
        self.manager = manager
        # Ring buffer of snapshots. Each snapshot is a dict.
        # Max length: 1000 entries. Oldest are popped from front.
        self._history = []

    # -------------------------------------------------------------------------
    # snapshot() -> dict
    # -------------------------------------------------------------------------
    # Captures the current system state and appends it to _history.
    #
    # Returns a dict with:
    #   timestamp – time.perf_counter() value (seconds since boot, high-res)
    #   tasks     – dict from scheduler.stats() (task Hz, exec time, jitter)
    #   health    – dict from health.status (component -> "alive"/"dead")
    #   perf      – dict from perf.report() (cpu_avg, ram_avg)
    #
    # Ring buffer: if _history exceeds 1000 entries, the oldest is removed.
    # This prevents unbounded memory growth during long runs.
    #
    # Called by: not currently called (manual or future diagnostics_task).
    # -------------------------------------------------------------------------
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
        # Enforce max history size (1000 entries)
        if len(self._history) > 1000:
            self._history.pop(0)
        return snap

    # -------------------------------------------------------------------------
    # dump() -> str (JSON)
    # -------------------------------------------------------------------------
    # Returns the last 100 snapshots as a pretty-printed JSON string.
    # Useful for writing to a log file on shutdown or for live inspection.
    #
    # The limit of 100 (not the full 1000) is a trade-off: 100 snapshots
    # is usually enough to diagnose recent issues without producing
    # enormous JSON output.
    # -------------------------------------------------------------------------
    def dump(self):
        import json
        return json.dumps(self._history[-100:], indent=2)
