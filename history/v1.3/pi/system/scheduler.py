import time
import asyncio
from collections import OrderedDict
from .logger import log

class Task:
    def __init__(self, name, callback, hz, priority=0):
        self.name = name
        self.callback = callback
        self.period = 1.0 / hz
        self.priority = priority
        self.last_run = 0
        self.jitter = 0.0
        self.total_runs = 0
        self.max_jitter = 0.0
        self.avg_exec_time = 0.0

class TaskScheduler:
    def __init__(self):
        self.tasks = OrderedDict()
        self._running = False

    def add(self, name, callback, hz, priority=0):
        self.tasks[name] = Task(name, callback, hz, priority)
        return self

    def remove(self, name):
        self.tasks.pop(name, None)

    async def spin_once(self):
        now = time.perf_counter()
        for task in sorted(self.tasks.values(), key=lambda t: t.priority, reverse=True):
            if now - task.last_run >= task.period:
                t0 = time.perf_counter()
                task.jitter = t0 - task.last_run - task.period
                task.max_jitter = max(task.max_jitter, abs(task.jitter))
                try:
                    if asyncio.iscoroutinefunction(task.callback):
                        await task.callback()
                    else:
                        task.callback()
                except Exception as e:
                    log.error(f"Task {task.name}: {e}")
                dt = time.perf_counter() - t0
                task.avg_exec_time = 0.95 * task.avg_exec_time + 0.05 * dt
                task.last_run = now
                task.total_runs += 1

    async def run(self):
        self._running = True
        while self._running:
            await self.spin_once()
            await asyncio.sleep(0)

    def stop(self):
        self._running = False

    def stats(self):
        return {
            name: {
                "hz": round(1.0 / t.period, 1),
                "avg_exec_ms": round(t.avg_exec_time * 1000, 3),
                "max_jitter_ms": round(t.max_jitter * 1000, 3),
                "runs": t.total_runs,
            }
            for name, t in self.tasks.items()
        }
