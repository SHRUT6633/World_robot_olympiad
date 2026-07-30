import gc
from .logger import log

class MemoryManager:
    def __init__(self, warning_mb=200, critical_mb=100):
        self.warning_mb = warning_mb
        self.critical_mb = critical_mb

    def collect(self):
        before = gc.get_count()
        gc.collect()
        after = gc.get_count()
        log.debug(f"GC: {before} -> {after}")

    def auto_collect(self):
        import psutil
        mem = psutil.virtual_memory()
        if mem.available < self.critical_mb * 1024 * 1024:
            self.collect()
            log.warn(f"Critical memory: {mem.available / 1024 / 1024:.0f}MB free")
        elif mem.available < self.warning_mb * 1024 * 1024:
            self.collect()
            log.info(f"Low memory: {mem.available / 1024 / 1024:.0f}MB free")
