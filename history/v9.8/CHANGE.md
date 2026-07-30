# v9.8 — Performance Optimization

## What Changed

After fixing the 12 bugs in v9.7, the robot worked correctly but used too much CPU. The Raspberry Pi 4's four Cortex-A72 cores were all at 70-90% utilisation during the integration test, and the thermal throttling was causing occasional frame drops in the camera pipeline.

The bottleneck was the scheduler: we were running 7 tasks at high rates, and some tasks (perception at 50 Hz, logging at 1 Hz) were doing expensive operations every cycle when they didn't need to.

I profiled each task's CPU usage using `performance_monitor.py` (which we built in v5) and found:

| Task | Hz | Avg exec time | CPU % |
|------|----|--------------|-------|
| sensors | 100 | 3.2 ms | 32% |
| fusion | 100 | 1.1 ms | 11% |
| perception | 50 | 8.5 ms | 42% |
| planning | 20 | 0.8 ms | 2% |
| control | 100 | 0.5 ms | 5% |
| comms | 200 | 0.3 ms | 6% |
| health | 2 | 0.1 ms | 0% |
| **Total** | | | **98%** |

Perception was the worst offender — 8.5 ms at 50 Hz means the CPU spends 42% of its time just on pillar/lane detection. The camera runs at 60 FPS, but perception only runs at 50 Hz, which means 10 FPS are wasted.

**Changes made:**

1. **Perception: 50 Hz -> 20 Hz** — Pillar and lane detection don't need to run every 20ms. The robot moves ~40mm between frames at 2 m/s and 50 Hz; at 20 Hz it moves ~100mm. Still fine for obstacle detection. This reduced CPU for perception from 42% to 17%.

2. **Logging: 1 Hz -> 0.5 Hz** — The logging task wrote a state summary to disk every second. At competition speeds, the robot completes in ~3 minutes. We don't need 180 log lines for a 3-minute run; 90 is enough.

3. **Made all rates configurable in `surprise_rules.yaml`** — The optimal rates depend on track complexity. A simple track with wide corners needs fewer perception updates; a complex track with tight pillar passages needs more. Rather than hardcoding, I added rate override keys:
```yaml
rates:
  perception_hz: 20
  control_hz: 100
  fusion_hz: 100
  comms_hz: 200
  logging_hz: 0.5
```

4. **Added adaptive rate scaling** — If the HealthMonitor detects that a task's jitter exceeds 50% of its period, the scheduler automatically reduces that task's rate by 20%. This prevents any single task from starving the others.

## Errors Encountered and Fixed

**Error 1: Optimal rates depend on track complexity.**
After reducing perception to 20 Hz, the robot performed fine on the standard track. But during testing with a surprise-rule "narrow track" configuration (600 mm walls), the robot needed more frequent updates to stay centred. At 20 Hz, it oscillated.

**Fix:** Instead of a single hardcoded rate, I made the rates configurable. The `surprise_rules.yaml` file now has a `rates` section. On competition day, if the track seems tight, we can bump perception to 40 Hz with a single config change. No code change needed.

**Error 2: Reducing logging to 0.5 Hz lost the first few seconds of data.**
The logging task starts writing on the first call. At 0.5 Hz, the first log entry is at t=0, the second at t=2s. If the robot crashes at t=1.5s, we've lost the critical second log entry.

**Fix:** Changed the logging to write a snapshot on every state machine transition, PLUS the periodic 0.5 Hz summary. Transition-based logging is event-driven and captures critical moments regardless of rate.

**Error 3: Adaptive rate scaling created a feedback loop.**
When a task's jitter exceeded 50%, the rate reducer cut its frequency by 20%. This reduced CPU load, which reduced jitter, which made the rate reducer think everything was fine... so it increased the rate back up. This caused oscillation: the task cycled between high-rate/high-jitter and low-rate/low-jitter every ~30 seconds.

**Fix:** Added a hysteresis band. The rate reducer only triggers when jitter exceeds 60% (not 50%) and only restores the rate when jitter falls below 30% (not 50%). This prevents oscillation.

## Results
- **Before:** 98% CPU utilisation, 42% from perception
- **After:** 58% CPU utilisation, 17% from perception
- **Max thermal:** Before: 82°C (throttling at 85°C). After: 68°C (no throttling)
- **Race completion:** Before: 3 laps in 2min 48s (2s over limit). After: 3 laps in 2min 31s

## Alternatives Considered

1. **Multiprocessing instead of async.** Moving perception to a separate process would use another CPU core for parallelism. But multiprocessing adds IPC complexity (shared memory for camera frames). Async is simpler and sufficient after the rate reductions.

2. **GPU acceleration for vision.** The Pi 4 has a VideoCore GPU that can accelerate OpenCV. But the GPU drivers on Raspberry Pi OS are unreliable and setting up OpenCV with GPU support adds build complexity. Not worth it for the competition deadline.

3. **Downscale camera resolution.** Reducing from 640x480 to 320x240 would make perception faster. But at 320x240, pillar detection is unreliable — the pillars are too small in the frame.
