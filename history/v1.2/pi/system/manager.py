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
        self.config = ConfigManager()
        self.config.load()
        self.scheduler = TaskScheduler()
        self.health = HealthMonitor()
        self.perf = PerformanceMonitor()
        self._running = False
        self._components = {}

    def register(self, name, component):
        self._components[name] = component
        log.info(f"Registered component: {name}")

    def get(self, name):
        return self._components.get(name)

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

    async def run(self):
        self._running = True
        log.info("System running")

        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, lambda: asyncio.create_task(self.stop()))
            except NotImplementedError:
                pass

        try:
            await self.scheduler.run()
        except asyncio.CancelledError:
            pass
        finally:
            await self.shutdown()

    async def stop(self):
        self._running = False
        self.scheduler.stop()
        self.perf.stop()
        log.info("System stopping...")

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
