# =============================================================================
# scheduler.py — TaskScheduler (Cooperative Async Task Runner)
# =============================================================================
# Implements a simple, deterministic task scheduler for running multiple
# async coroutines at fixed frequencies with priority ordering.
#
# Design:
#   - The run() method loops as fast as asyncio allows, calling spin_once()
#     on each iteration.
#   - spin_once() iterates over all registered tasks, sorted by priority
#     (highest first). For each task, if its period (1/hz) has elapsed
#     since last run, the task's callback is executed (awaited).
#   - The scheduler is cooperative: it relies on each callback returning
#     control via await asyncio.sleep(0) or similar. If a callback blocks
#     for a long time, every task behind it in that tick is delayed.
#   - Priority only matters within a single tick. Lower-priority tasks still
#     execute at their designated Hz rate — they just run after higher-priority
#     ones when multiple tasks are due in the same tick.
#
# Key metrics tracked per task:
#   last_run     – timestamp of the most recent execution
#   jitter       – how late (or early) this run was relative to the ideal period
#   max_jitter   – worst-case absolute jitter observed
#   avg_exec_time – exponential moving average of execution duration
#   total_runs   – total number of times this task has been executed
#
# Who creates the TaskScheduler?
#   - SystemManager.__init__()  (manager.py line 14)
#
# Who adds tasks?
#   - pi/main.py lines 176-182: mgr.scheduler.add(...) for each subsystem
#
# Who calls run()?
#   - SystemManager.run() awaits scheduler.run() (manager.py line 54)
#
# Impact of changing Hz values:
#   - Increasing Hz → shorter period, task runs more often, more CPU load.
#     If the callback takes longer than the period, the task effectively
#     runs at a lower rate (it will always be "due" by the time it finishes).
#   - Decreasing Hz → longer period, less CPU, but slower response.
#   - Example: control at 100 Hz → period = 10 ms. If compute_steering()
#     takes 8 ms, there are only 2 ms of slack before the next tick.
#
# Impact of changing priority:
#   - Higher priority tasks run first in each tick. If a high-priority task
#     takes a long time, lower-priority tasks that are also due may be
#     delayed by a few microseconds (but still within Hz bounds).
#   - Priority is most noticeable when multiple tasks are due at the exact
#     same moment (e.g., after a gap in the event loop).
# =============================================================================

import time
import asyncio
from collections import OrderedDict
from .logger import log


# =============================================================================
# Task — Internal data class for a single scheduled task
# =============================================================================
class Task:
    def __init__(self, name, callback, hz, priority=0):
        # Unique task name (used for logging, stats, and health tracking).
        self.name = name
        # The async or sync callable to execute.
        self.callback = callback
        # Period in seconds = 1 / hz. E.g., 100 Hz → 0.01 s.
        self.period = 1.0 / hz
        # Priority: higher = runs first within a scheduler tick.
        self.priority = priority
        # Timestamp (from time.perf_counter()) of the last execution.
        self.last_run = 0
        # Jitter = actual_time - expected_time.
        # Positive means late, negative means early (rare).
        self.jitter = 0.0
        # Total number of times the task has been executed.
        self.total_runs = 0
        # Worst-case absolute jitter ever recorded.
        self.max_jitter = 0.0
        # Exponential moving average of execution time (seconds).
        # Updated as: avg = 0.95 * avg + 0.05 * latest_dt
        # This smooths out spikes and gives a stable estimate.
        self.avg_exec_time = 0.0


# =============================================================================
# TaskScheduler — Main scheduler class
# =============================================================================
class TaskScheduler:
    def __init__(self):
        # OrderedDict preserves insertion order.
        # Tasks are stored by name for O(1) lookup in remove() / stats().
        self.tasks = OrderedDict()
        # Flag to stop the run loop.
        self._running = False

    # -------------------------------------------------------------------------
    # add(name, callback, hz, priority=0)
    # -------------------------------------------------------------------------
    # Registers a new task. Returns self for method chaining.
    #
    # Parameters:
    #   name     – Unique string identifier.
    #   callback – Async or sync function to call.
    #   hz       – Target frequency (Hz). Period = 1 / hz.
    #   priority – Relative priority (higher = sooner in each tick).
    #
    # If a task with the same name already exists, it is overwritten.
    # -------------------------------------------------------------------------
    def add(self, name, callback, hz, priority=0):
        self.tasks[name] = Task(name, callback, hz, priority)
        return self

    # -------------------------------------------------------------------------
    # remove(name)
    # -------------------------------------------------------------------------
    # Removes a task by name. No-op if name is not found.
    # -------------------------------------------------------------------------
    def remove(self, name):
        self.tasks.pop(name, None)

    # -------------------------------------------------------------------------
    # spin_once()
    # -------------------------------------------------------------------------
    # Executes one scheduler tick:
    #   1. Record current time.
    #   2. Iterate tasks sorted by priority descending.
    #   3. For each task whose period has elapsed:
    #        a. Calculate jitter = (current_time - last_run - period)
    #        b. Track max_jitter = max(max_jitter, abs(jitter))
    #        c. Execute the callback (await if async, call if sync).
    #        d. Catch and log any exceptions (task does NOT crash the system).
    #        e. Update avg_exec_time (EMA) and total_runs.
    #
    # Note: If a callback raises an exception, it is logged but the scheduler
    #       continues to the next task. No tasks are ever removed due to
    #       errors — the health monitor may detect the "dead" component later.
    #
    # Called by: run() in a tight loop.
    # -------------------------------------------------------------------------
    async def spin_once(self):
        now = time.perf_counter()
        # Sort tasks by priority descending so high-priority tasks run first.
        for task in sorted(self.tasks.values(), key=lambda t: t.priority, reverse=True):
            if now - task.last_run >= task.period:
                # Time to run this task.
                t0 = time.perf_counter()
                # Jitter: positive means the task is running later than ideal.
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
                # EMA: 95% old average + 5% new sample (smoothing factor = 0.05).
                task.avg_exec_time = 0.95 * task.avg_exec_time + 0.05 * dt
                task.last_run = now
                task.total_runs += 1

    # -------------------------------------------------------------------------
    # run()
    # -------------------------------------------------------------------------
    # Main loop: continuously calls spin_once(), then yields control to the
    # asyncio event loop via await asyncio.sleep(0). This allows other async
    # tasks (not managed by the scheduler) to run between ticks.
    #
    # Exits when _running is set to False (by stop()).
    # -------------------------------------------------------------------------
    async def run(self):
        self._running = True
        while self._running:
            await self.spin_once()
            await asyncio.sleep(0)  # Yield to event loop

    # -------------------------------------------------------------------------
    # stop()
    # -------------------------------------------------------------------------
    # Sets the running flag to False. The run() loop will exit on the next
    # iteration.
    # -------------------------------------------------------------------------
    def stop(self):
        self._running = False

    # -------------------------------------------------------------------------
    # stats()
    # -------------------------------------------------------------------------
    # Returns a dictionary of per-task statistics, useful for diagnostics:
    #   {
    #     "task_name": {
    #       "hz":           round(1/period, 1),      # target frequency
    #       "avg_exec_ms":  avg_exec_time * 1000,    # average runtime in ms
    #       "max_jitter_ms": max_jitter * 1000,      # worst jitter in ms
    #       "runs":         total_runs               # total executions
    #     }
    #   }
    #
    # Called by:
    #   - diagnostics.py snapshot() method to include in diagnostic dumps.
    # -------------------------------------------------------------------------
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
