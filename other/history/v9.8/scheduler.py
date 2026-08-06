import time
import asyncio
from collections import OrderedDict
from .logger import log


class Task:
    def __init__(self, name, callback, hz, priority=0):
        self.name = name
        self.callback = callback
        self.base_hz = hz
        self.current_hz = hz
        self.period = 1.0 / hz
        self.priority = priority
        self.last_run = 0
        self.jitter = 0.0
        self.total_runs = 0
        self.max_jitter = 0.0
        self.avg_exec_time = 0.0
        self._jitter_samples = []

    def rate_reduce(self):
        self.current_hz = max(self.current_hz * 0.8, 1.0)
        self.period = 1.0 / self.current_hz

    def rate_restore(self):
        self.current_hz = self.base_hz
        self.period = 1.0 / self.current_hz


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
        for task in sorted(self.tasks.values(),
                           key=lambda t: t.priority, reverse=True):
            if now - task.last_run >= task.period:
                t0 = time.perf_counter()
                task.jitter = t0 - task.last_run - task.period
                task.max_jitter = max(task.max_jitter, abs(task.jitter))
                task._jitter_samples.append(abs(task.jitter))
                if len(task._jitter_samples) > 20:
                    task._jitter_samples.pop(0)
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

    def _apply_adaptive_rates(self):
        for task in self.tasks.values():
            if task.base_hz <= 1.0:
                continue
            avg_jitter = (sum(task._jitter_samples) /
                          max(len(task._jitter_samples), 1))
            period = task.period
            jitter_ratio = avg_jitter / period if period > 0 else 0
            if jitter_ratio > 0.6 and task.current_hz > 1.0:
                task.rate_reduce()
                log.info(f"Rate reduce: {task.name} -> {task.current_hz} Hz")
            elif jitter_ratio < 0.3 and task.current_hz < task.base_hz:
                task.rate_restore()
                log.info(f"Rate restore: {task.name} -> {task.current_hz} Hz")

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
                "hz": round(t.current_hz, 1),
                "base_hz": round(t.base_hz, 1),
                "avg_exec_ms": round(t.avg_exec_time * 1000, 3),
                "max_jitter_ms": round(t.max_jitter * 1000, 3),
                "runs": t.total_runs,
            }
            for name, t in self.tasks.items()
        }
