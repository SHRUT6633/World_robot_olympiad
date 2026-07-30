# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/system/memory_manager.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Automatic GC and memory-pressure monitor
# =============================================================================

# =============================================================================
# memory_manager.py — Automatic garbage collection & memory-pressure monitor
# =============================================================================
# This module monitors system memory and triggers Python GC when free memory
# drops below configurable thresholds.  It is used by the main robot loop to
# prevent out-of-memory crashes during prolonged operation.
# =============================================================================

import gc                                   # Python's garbage-collector interface
from .logger import log                     # Project-wide logger (relative import)


class MemoryManager:
    """
    Periodic memory-pressure monitor.

    Attributes
    ----------
    warning_mb  : int
        Soft threshold (MB) — GC is triggered and an info message is logged.
    critical_mb : int
        Hard threshold (MB) — GC is triggered and a **warning** is logged.
    """

    def __init__(self, warning_mb=200, critical_mb=100):
        # ----- Configuration -----
        self.warning_mb = warning_mb          # Soft limit (default 200 MB)
        self.critical_mb = critical_mb        # Hard limit (default 100 MB)

    # ------------------------------------------------------------------
    # collect — Force an immediate full GC cycle
    # ------------------------------------------------------------------
    def collect(self):
        """
        Run gc.collect() and log how the tracked-object count changed.

        This is useful for diagnosing memory leaks: if 'after' keeps growing
        the same object type is probably being retained unintentionally.
        """
        before = gc.get_count()               # Number of tracked objects *before*
        gc.collect()                          # Perform full collection
        after = gc.get_count()                # Number of tracked objects *after*
        log.debug(f"GC: {before} -> {after}")  # Log delta at DEBUG level

    # ------------------------------------------------------------------
    # auto_collect — Conditionally collect based on free memory
    # ------------------------------------------------------------------
    def auto_collect(self):
        """
        Check free physical memory via psutil and call collect() if either
        the warning or critical threshold has been breached.

        Effects of changing thresholds:
        - Lower  *warning_mb* → GC fires less often (saves CPU, risks OOM).
        - Higher *warning_mb* → GC fires more often (safer, more CPU overhead).
        - *critical_mb* should always be ≤ *warning_mb*.
        """
        import psutil                         # Third-party: system-info library
        mem = psutil.virtual_memory()         # Named tuple with total/used/available/...
        # --- Check the hard (critical) limit first ---
        if mem.available < self.critical_mb * 1024 * 1024:
            self.collect()                    # Free what we can
            log.warn(f"Critical memory: {mem.available / 1024 / 1024:.0f}MB free")
        # --- Then check the soft (warning) limit ---
        elif mem.available < self.warning_mb * 1024 * 1024:
            self.collect()
            log.info(f"Low memory: {mem.available / 1024 / 1024:.0f}MB free")
        # Note: no 'else' — if memory is healthy we do nothing.
