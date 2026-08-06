import asyncio
import logging
from dataclasses import dataclass, field
from collections import defaultdict

logger = logging.getLogger("scheduler")


@dataclass
class TaskSpec:
    name: str
    coro_func: callable
    rate_hz: float
    priority: int = 0
    deadline_slack_pct: float = 20.0


@dataclass
class TaskStats:
    name: str
    iterations: int = 0
    total_runtime_s: float = 0.0
    max_runtime_s: float = 0.0
    last_runtime_s: float = 0.0
    deadline_misses: int = 0


class MultiRateScheduler:
    def __init__(self):
        self._tasks: list[TaskSpec] = []
        self._stats: dict[str, TaskStats] = {}
        self._running = False

    def add_task(self, spec: TaskSpec):
        self._tasks.append(spec)
        self._stats[spec.name] = TaskStats(name=spec.name)

    async def run(self):
        self._running = True
        loop = asyncio.get_event_loop()
        runners = [self._run_task(spec, loop) for spec in self._tasks]
        await asyncio.gather(*runners)

    async def stop(self):
        self._running = False

    async def _run_task(self, spec: TaskSpec, loop: asyncio.AbstractEventLoop):
        period = 1.0 / spec.rate_hz
        next_time = loop.time()
        deadline = period * (1.0 - spec.deadline_slack_pct / 100.0)
        stats = self._stats[spec.name]

        while self._running:
            t0 = loop.time()
            try:
                await spec.coro_func()
            except Exception:
                logger.exception(f"Task {spec.name} raised exception")
            t1 = loop.time()
            elapsed = t1 - t0

            stats.iterations += 1
            stats.total_runtime_s += elapsed
            stats.last_runtime_s = elapsed
            stats.max_runtime_s = max(stats.max_runtime_s, elapsed)

            if elapsed > deadline:
                stats.deadline_misses += 1
                logger.warning(
                    f"Task {spec.name} missed deadline: "
                    f"{elapsed*1000:.1f}ms > {deadline*1000:.1f}ms"
                )

            next_time += period
            delay = next_time - loop.time()
            if delay > 0:
                await asyncio.sleep(delay)

    def get_stats(self) -> dict[str, TaskStats]:
        return dict(self._stats)

    def print_summary(self):
        for name, s in self._stats.items():
            avg = s.total_runtime_s / max(s.iterations, 1)
            logger.info(
                f"{name}: {s.iterations} iters, "
                f"avg {avg*1000:.2f}ms, max {s.max_runtime_s*1000:.2f}ms, "
                f"missed {s.deadline_misses}"
            )
