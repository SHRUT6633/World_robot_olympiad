# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/system/health_monitor.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Heartbeat-based liveness detection
# =============================================================================

# =============================================================================
# health_monitor.py — HealthMonitor (Heartbeat-Based Liveness Detection)
# =============================================================================
# The HealthMonitor keeps track of whether each subsystem is "alive" by
# expecting periodic heartbeat calls. If a component stops calling
# heartbeat(), its last heartbeat timestamp ages, and when it exceeds
# timeout_s, the component is marked "dead".
#
# This is a passive, non-intrusive monitor:
#   - Components call heartbeat() in their main loop (e.g., sensor_task
#     calls mgr.health.heartbeat("sensors") every 10 ms).
#   - The health_checker task (in main.py) calls check_all() at 2 Hz
#     and logs warnings for dead components.
#   - No automatic recovery is attempted — the monitor is purely
#     diagnostic. The robot continues running even with dead components.
#
# Key concept:
#   heartbeat("sensors") is called every 10 ms → last_heartbeat["sensors"]
#   is updated. check("sensors") compares time.perf_counter() - last
#   against timeout_s (default 2.0 s). If the difference exceeds 2 s,
#   the component is "dead". This means a component has 2 seconds of
#   slack before being declared dead — useful for tasks that may have
#   occasional long runs (e.g., perception at 50 Hz).
#
# Why a separate monitor instead of just relying on the scheduler?
#   The scheduler only tracks execution timing. It doesn't distinguish
#   between a task that takes too long and a task that crashed silently.
#   HealthMonitor explicitly tracks "did this component call heartbeat()?"
#   If a task crashes, its try/except in spin_once() logs the error but
#   the scheduler continues. The heartbeat would stop, and health check
#   would detect the silence.
#
# Impact of changing timeout_s:
#   timeout_s = 2.0 → A component must miss ~200 calls at 100 Hz to be
#   declared dead. This is lenient and avoids false positives from
#   occasional scheduling hiccups.
#   timeout_s = 0.5 → More sensitive; any 500 ms stall triggers a warning.
#   timeout_s = 10.0 → Very lenient; a dead component may go unnoticed
#   for 10 seconds.
#
# Who uses HealthMonitor?
#   - SystemManager.health (the instance).
#   - Each async task in main.py calls mgr.health.heartbeat(name).
#   - The health_task in main.py calls mgr.health.check_all() and logs.
#   - diagnostics.py includes health status in its snapshots.
#   - manager.py shutdown() logs health.summary().
# =============================================================================

import time
from collections import defaultdict
from .logger import log


class HealthMonitor:
    def __init__(self, timeout_s=2.0):
        # timeout_s: Maximum allowed seconds since the last heartbeat
        # before a component is considered dead. Default 2.0 seconds.
        # Change this to adjust sensitivity to task stalls.
        self.timeout_s = timeout_s

        # _last_heartbeat: dict[component_name -> time.perf_counter()]
        # Stores the most recent heartbeat timestamp for each component.
        self._last_heartbeat = {}

        # _status: dict[component_name -> "alive" | "dead"]
        # Cached status from the most recent check() call.
        self._status = {}

    # -------------------------------------------------------------------------
    # heartbeat(component: str)
    # -------------------------------------------------------------------------
    # Called by a component to signal "I am alive."
    # Updates the last heartbeat timestamp to now and sets status to "alive".
    #
    # Called by:
    #   - sensor_task()     → mgr.health.heartbeat("sensors")
    #   - fusion_task()     → mgr.health.heartbeat("fusion")
    #   - perception_task() → mgr.health.heartbeat("perception")
    #   - planning_task()   → mgr.health.heartbeat("planning")
    #   - control_task()    → mgr.health.heartbeat("control")
    #   - comm_task()       → mgr.health.heartbeat("comm")
    #
    # If a component never calls heartbeat(), it will never appear in
    # check_all() results (it is simply unknown).
    # -------------------------------------------------------------------------
    def heartbeat(self, component: str):
        self._last_heartbeat[component] = time.perf_counter()
        self._status[component] = "alive"

    # -------------------------------------------------------------------------
    # check(component: str) -> bool
    # -------------------------------------------------------------------------
    # Returns True if the component is alive (last heartbeat within timeout),
    # False if it is dead (heartbeat too old or never received).
    #
    # This is called internally by check_all().
    # -------------------------------------------------------------------------
    def check(self, component: str) -> bool:
        last = self._last_heartbeat.get(component, 0)
        dt = time.perf_counter() - last
        if dt > self.timeout_s:
            self._status[component] = "dead"
            return False
        self._status[component] = "alive"
        return True

    # -------------------------------------------------------------------------
    # check_all() -> dict
    # -------------------------------------------------------------------------
    # Checks every component that has ever called heartbeat() and returns
    # a dict: {component_name: True/False}.
    #
    # Called by: health_task in main.py every 0.5 seconds.
    # -------------------------------------------------------------------------
    def check_all(self):
        results = {}
        for comp in self._last_heartbeat:
            results[comp] = self.check(comp)
        return results

    # -------------------------------------------------------------------------
    # status (property)
    # -------------------------------------------------------------------------
    # Returns a copy of the current status dict (for diagnostics).
    # -------------------------------------------------------------------------
    @property
    def status(self):
        return dict(self._status)

    # -------------------------------------------------------------------------
    # summary() -> str
    # -------------------------------------------------------------------------
    # Returns a human-readable summary like "5/6 components alive".
    # Called by: SystemManager.shutdown() for final log message.
    # -------------------------------------------------------------------------
    def summary(self):
        alive = sum(1 for s in self._status.values() if s == "alive")
        dead = sum(1 for s in self._status.values() if s == "dead")
        return f"{alive}/{len(self._status)} components alive"
