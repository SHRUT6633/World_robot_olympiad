# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/system/manager.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# System orchestrator
# =============================================================================

# =============================================================================
# manager.py — SystemManager (Orchestrator)
# =============================================================================
# The SystemManager is the central orchestrator for the entire robot software
# stack. It owns and coordinates:
#
#   ConfigManager   — loads and provides runtime configuration values
#   TaskScheduler   — runs async tasks at specified frequencies
#   HealthMonitor   — tracks heartbeat from each subsystem
#   PerformanceMonitor — logs CPU/RAM usage periodically
#
# Responsibilities:
#   1. Component Registry: register(name, component) / get(name)
#      → Tracks all major modules so their lifecycle (init / close) is
#        managed uniformly.
#   2. init_all(): Iterates over all registered components, calling .init()
#      if the method exists (supports both sync and async init).
#   3. run(): Starts the scheduler loop. On SIGINT/SIGTERM, triggers a
#      graceful shutdown.
#   4. stop() / shutdown(): Stops scheduler + perf monitor, then calls
#      .close() on every component in reverse registration order (so
#      dependencies are closed after their consumers).
#
# Who instantiates SystemManager?
#   - pi/main.py line 39:  mgr = SystemManager()
#   - It is created ONCE per race run.
#
# Connection to other files:
#   - config_manager.py  → self.config (singleton)
#   - scheduler.py       → self.scheduler (TaskScheduler)
#   - health_monitor.py  → self.health  (HealthMonitor)
#   - performance_monitor.py → self.perf (PerformanceMonitor)
#   - logger.py          → shared global `log` for all messages
# =============================================================================

import asyncio
import signal
import sys
from .config_manager import ConfigManager
from .logger import log
from .scheduler import TaskScheduler
from .health_monitor import HealthMonitor
from .performance_monitor import PerformanceMonitor


class SystemManager:
    def __init__(self):
        # ConfigManager is a singleton — ConfigManager() always returns the
        # same instance. .load() reads pi_config.yaml into its internal dict.
        self.config = ConfigManager()
        self.config.load()

        # TaskScheduler manages the list of async tasks and their periods.
        self.scheduler = TaskScheduler()

        # HealthMonitor tracks heartbeats from tasks/components.
        # Default timeout: 2.0 seconds (configurable via HealthMonitor arg).
        # If a heartbeat is older than timeout_s, the component is "dead".
        self.health = HealthMonitor()

        # PerformanceMonitor logs CPU% and RAM% on a background thread.
        self.perf = PerformanceMonitor()

        # Internal state flag — set True when run() starts, False on stop().
        self._running = False

        # Component registry: dict[name_str -> component_object].
        # Components are added via register() and iterated during init_all()
        # and shutdown().
        self._components = {}

    # -------------------------------------------------------------------------
    # register(name, component)
    # -------------------------------------------------------------------------
    # Adds a component to the registry under a string name.
    # The name is used for:
    #   - Log messages ("Registered component: {name}")
    #   - Health heartbeat keys (e.g. HealthMonitor.heartbeat("fusion"))
    #   - Shutdown ordering (reversed dict order)
    # -------------------------------------------------------------------------
    def register(self, name, component):
        self._components[name] = component
        log.info(f"Registered component: {name}")

    # -------------------------------------------------------------------------
    # get(name)
    # -------------------------------------------------------------------------
    # Retrieves a registered component by name. Returns None if not found.
    # Called by other modules that need a reference to a specific component
    # (e.g., diagnostics.py uses this to access scheduler/health/perf).
    # -------------------------------------------------------------------------
    def get(self, name):
        return self._components.get(name)

    # -------------------------------------------------------------------------
    # init_all()
    # -------------------------------------------------------------------------
    # Iterates over all registered components. For each one:
    #   1. Checks if it has a callable .init() method.
    #   2. If .init() is a coroutine function, awaits it.
    #   3. If .init() is synchronous, calls it directly.
    #   4. Logs success or failure for each component.
    # After all components are initialized, starts the PerformanceMonitor
    # background thread.
    #
    # This ensures that hardware is powered up and software modules are ready
    # before the main loop begins.
    # -------------------------------------------------------------------------
    async def init_all(self):
        log.info("Initializing all components...")
        for name, comp in self._components.items():
            try:
                if hasattr(comp, "init") and callable(comp.init):
                    if asyncio.iscoroutinefunction(comp.init):
                        await comp.init()
                    else:
                        comp.init()
                log.info(f"  {name}: OK")
            except Exception as e:
                log.error(f"  {name}: FAILED - {e}")
        self.perf.start()
        log.info("System initialization complete")

    # -------------------------------------------------------------------------
    # run()
    # -------------------------------------------------------------------------
    # Sets _running = True, registers signal handlers for SIGINT and SIGTERM
    # (to gracefully stop on Ctrl+C or kill), then enters the scheduler loop.
    # When the scheduler exits (via CancelledError or stop()), calls shutdown()
    # to clean up all components.
    #
    # Signal handler: On Unix, add_signal_handler works. On Windows,
    # NotImplementedError is raised and silently ignored — KeyboardInterrupt
    # is the fallback.
    # -------------------------------------------------------------------------
    async def run(self):
        self._running = True
        log.info("System running")

        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                # On SIGINT/SIGTERM, schedule an async stop() call.
                loop.add_signal_handler(sig, lambda: asyncio.create_task(self.stop()))
            except NotImplementedError:
                # Windows does not support add_signal_handler.
                pass

        try:
            await self.scheduler.run()
        except asyncio.CancelledError:
            # Raised when tasks are cancelled during shutdown.
            pass
        finally:
            await self.shutdown()

    # -------------------------------------------------------------------------
    # stop()
    # -------------------------------------------------------------------------
    # Signals all subsystems to stop:
    #   - Sets _running = False so the scheduler loop exits.
    #   - Calls scheduler.stop() which sets TaskScheduler._running = False.
    #   - Calls perf.stop() which stops the background monitoring thread.
    # This is called either by the signal handler or by the KeyboardInterrupt
    # catch in main.py (pi/main.py line 192).
    # -------------------------------------------------------------------------
    async def stop(self):
        self._running = False
        self.scheduler.stop()
        self.perf.stop()
        log.info("System stopping...")

    # -------------------------------------------------------------------------
    # shutdown()
    # -------------------------------------------------------------------------
    # Gracefully shuts down all registered components by calling .close() on
    # each, in reverse registration order. This ensures that components which
    # depend on others (e.g., a filter that depends on a sensor) are closed
    # before their dependencies.
    #
    # After closing all components, logs a health summary and a performance
    # report so that post-race analysis can check for anomalies.
    # -------------------------------------------------------------------------
    async def shutdown(self):
        for name, comp in reversed(list(self._components.items())):
            try:
                if hasattr(comp, "close") and callable(comp.close):
                    if asyncio.iscoroutinefunction(comp.close):
                        await comp.close()
                    else:
                        comp.close()
            except Exception as e:
                log.warn(f"{name} close: {e}")
        log.info(f"Health: {self.health.summary()}")
        log.info(f"Perf: {self.perf.report()}")
        log.info("Shutdown complete")
