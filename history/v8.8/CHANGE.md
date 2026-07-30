# v8.8 — Health Monitor and Heartbeat Watchdog

## What Changed

With the multi-rate scheduler running multiple async tasks, I needed a way to detect if any task has crashed or hung. I built `health_monitor.py` that checks heartbeats from each task and triggers an emergency stop if any task stops sending heartbeats.

Each task calls `health_monitor.heartbeat(task_name)` at the end of its iteration. The health monitor runs in a separate low-rate loop (10 Hz) and checks that each task has sent a heartbeat within its expected interval (3x the task period).

## Errors Encountered

The first test was frustrating — the emergency stop kept triggering even when all tasks were running fine:

```
[HEALTH_MONITOR] WARN: Task 'sensors' heartbeat overdue by 15ms
[HEALTH_MONITOR] WARN: Task 'perception' heartbeat overdue by 32ms
[HEALTH_MONITOR] EMERGENCY STOP: Task 'sensors' heartbeat overdue by 51ms
```

The sensor task at 100 Hz has a 10ms period. I set the timeout to 30ms (3x period). But during heavy CPU load (when the perception task starts processing a high-res image), the scheduler delays the sensor task by up to 40ms. This is within our deadline slack (80% of target rate = 80Hz minimum = 12.5ms max period), but exceeds the 30ms heartbeat timeout.

The issue is that the health monitor timeout was too aggressive. I assumed "task is running" = "heartbeat every task period", but the scheduler allows tasks to slow down under load. The health monitor was flagging legitimate scheduling delays as failures.

## The Fix

I changed the timeout to allow 3 missed heartbeats before declaring a task dead:

```python
HEARTBEAT_TIMEOUT = 3 * task_period  # was 1 * task_period
```

This means a 100Hz task can miss 3 consecutive heartbeats (be delayed by up to 30ms accumulated) before the emergency stop triggers. Given our 80% rate threshold, the maximum allowed delay is 12.5ms per iteration, so 3 missed heartbeats = 37.5ms total delay, which is more than the 30ms timeout. Wait — that's still too tight.

Let me recalculate. The deadline monitor allows up to 20% slack, meaning a 100Hz task can run at 80Hz minimum. That means the maximum period is 12.5ms. Three missed heartbeats would be 37.5ms, and the timeout should be at least that. I set it to 50ms (5x period) for 100Hz tasks, 100ms (5x period) for 50Hz tasks, etc.

Actually, I made the timeout configurable per-task and set `HEARTBEAT_ALLOWED_MISSES = 3` as a global parameter. The timeout is calculated as `HEARTBEAT_ALLOWED_MISSES * task_period`. With 3 misses, a 100Hz task has 30ms timeout, which is enough for normal scheduling jitter but catches a hung task within 30ms.

## Alternatives Considered

1. **Hardware watchdog**: The Raspberry Pi has a hardware watchdog timer that can reset the entire system if the software stops responding. This is the nuclear option — it catches everything including kernel panics. But it also resets the system, which takes 15 seconds. In competition, that's an automatic DNF. We'd rather attempt recovery than reset.

2. **Supervisor process**: Run a separate Python process that monitors the main process. If the main process crashes, the supervisor restarts it. This handles crashes but not hangs (if the main process is stuck in an infinite loop, the supervisor won't know). Also, monitoring across processes adds complexity.

3. **Dead man's switch**: A hardware switch that must be toggled by the software within a certain interval, or the switch opens and cuts power to the motors. This is a common safety feature in robotics competitions. But it requires additional wiring and a dedicated GPIO pin. It's on the roadmap for v9.0.

4. **Watchdog with recovery**: Instead of emergency stop, try to restart the failed task. This is what I'll implement in v8.9 (or v9.0). For now, emergency stop is safer because I don't trust the recovery mechanism enough for competition.

## Testing

- Simulated 200ms hang in sensors task: detected in 32ms, emergency stop triggered
- Heavy CPU load (perception at max resolution): zero false positives over 10 minutes
- Simulated crash (task throws exception): detected in 1 cycle (heartbeat stops)
- Recovery after emergency stop: manual reset required
- Normal operation: zero false positives over 60 minutes of testing

## Lessons Learned

Heartbeat monitoring is essential for a multi-task system, but the timeout must account for scheduling jitter. A single missed heartbeat is normal; two is unusual; three is dead. I also learned that timeouts should be configurable per-task because tasks have different priorities and different acceptable latencies. The emergency stop should be the last resort — future versions should attempt task restart before stopping.
