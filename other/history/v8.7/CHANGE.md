# v8.7 — Multi-Rate Task Scheduler

## What Changed

The robot's software stack has grown to include multiple concurrent tasks with different timing requirements:

| Task       | Required Rate | Priority |
|------------|---------------|----------|
| Sensors    | 100 Hz        | Critical |
| Perception | 50 Hz         | High     |
| Control    | 100 Hz        | Critical |
| Logging    | 1 Hz          | Low      |

I built `scheduler.py` to run these tasks at their specified rates. The scheduler uses cooperative multitasking (async/await) because we can't afford the overhead of threads on the Raspberry Pi's limited cores.

## Errors Encountered

The first version used relative delays between iterations:

```python
async def run_task(task, rate):
    period = 1.0 / rate
    while True:
        await task.run()
        await asyncio.sleep(period)
```

This caused systematic drift. Each iteration took a small amount of time for the task itself plus scheduling overhead (~0.3ms). Over 30 minutes, the sensor task drifted by 0.54 seconds relative to the control task:

```
[SCHEDULER] INFO: Sensor task iteration 180000 — expected runtime: 30:00.000, actual: 30:00.540
[SCHEDULER] WARN: Sensor-control drift: 0.54s
[SCHEDULER] WARN: Perception-control drift: 1.12s
[CONTROL] ERROR: Sensor timestamp 180000 is 0.54s behind control timestamp 179946
```

The drift caused the control task to use sensor data that was 0.54 seconds old, which at 0.5 m/s means the robot's position estimate was 0.27m behind reality. This caused the robot to overshoot curves.

## The Fix

I switched to absolute time scheduling using `asyncio`'s event loop time:

```python
async def run_task(task, rate):
    period = 1.0 / rate
    next_time = event_loop.time()
    while True:
        await task.run()
        next_time += period
        delay = next_time - event_loop.time()
        if delay > 0:
            await asyncio.sleep(delay)
```

This way, the scheduler uses the wall clock to determine when the next iteration should start, regardless of how long the previous iteration took. If a task runs long, the next iteration is delayed (which is correct — we can't make up missed time), but the drift from scheduling overhead is eliminated.

I also added a deadline monitor that warns if a task's actual rate drops below 80% of the target rate.

## Alternatives Considered

1. **Thread-based scheduling**: Each task runs in its own thread with `time.sleep()`. This is simpler to implement and Python's GIL handles the timing reasonably well for I/O-bound tasks. But our perception task is CPU-bound (image processing), and threads would compete for the GIL, causing both tasks to slow down.

2. **Timer-based scheduling**: Use `asyncio.create_task()` with `asyncio.sleep()` for the period, similar to JavaScript's `setInterval()`. This is what I implemented first, but it has the same drift issue because `asyncio.sleep(period)` waits for `period` seconds from the time it's called, not from the start of the last iteration.

3. **Hardware timer interrupts**: Use the Raspberry Pi's PWM hardware to generate interrupts at precise intervals. This would give microsecond accuracy but requires C extensions and significantly complicates the code. Overkill for a 100Hz task.

4. **Priority-based preemption**: Instead of fixed rates, assign priorities and let higher-priority tasks preempt lower ones. This works in real-time OSes but not in Python's asyncio, which is inherently cooperative. A high-priority task that doesn't yield will starve lower-priority tasks.

## Testing

- 30-minute run: max drift 0.003s (3ms) across all tasks
- Sensor task: 179999 iterations (target 180000) — 99.999% accuracy
- Control task: 179999 iterations
- Perception task: 89999 iterations (target 90000) — 99.999% accuracy
- Max scheduling jitter: ±0.5ms
- Deadline violations: 0

The absolute time scheduling completely eliminates cumulative drift. Jitter is still present (due to Python's GC and async overhead) but it's bounded and doesn't accumulate.

## Lessons Learned

"The scheduler is the most important piece of software" — my controls professor was right. A 0.54s drift over 30 minutes doesn't sound bad, but at 100Hz it means the control loop is using data from 54 iterations ago. For a robot moving at 0.5 m/s, that's 0.27m of position error. Always use absolute time scheduling, not relative delays. And always monitor task rates — a single slow iteration can cascade into missed deadlines across the system.
