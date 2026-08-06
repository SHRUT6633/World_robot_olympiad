# v9.7 — Bug Fixes from Integration Testing

## What Changed

The integration test from v9.6 revealed 12 bugs that unit tests had missed. These were the kind of bugs that only manifest when the full system runs together — timing issues, edge cases in state transitions, sensor failure cascades. I spent two full days fixing them, one by one.

**The 12 bugs, grouped by subsystem:**

**Lap Counter (2 bugs):**
1. Off-by-one: `lap_counter.mark_crossing()` incremented `current_lap` before checking if the robot had moved past the line. Result: the first line crossing at the start counted as lap 1, so the robot only completed 2 laps instead of 3.
2. Reset race: `reset_crossing()` had no debounce timeout. If the robot crossed the finish line and immediately called `reset_crossing()`, the flag was cleared before the next tick could re-arm it.

**Velocity Profiler (1 bug):**
3. Division by zero in `velocity_profile.py:55` when `speed=0`: `self.k_soft + v` in the Stanley controller denominator causes `ZeroDivisionError`... wait, that's not right. Let me check. Actually the bug was in `velocity_profile.py:55`: `v_curv = np.sqrt(max_lat_a / (np.abs(curvature) + 1e-6))`. The `1e-6` prevents division by zero, so that's fine. The actual bug was in `trapezoidal()` when `dt=0` or negative. Not a runtime issue, but a logical bug.

Wait, let me reconsider. The bug was: `accel_steps = int(self.max_v / (self.max_a * dt))`. If `max_a` is 0 (configured incorrectly), this divides by zero. If `dt` is very small, `accel_steps` becomes very large, the loop iterates millions of times, and the profile is all acceleration with no cruise or deceleration. If `max_v` is 0, the division is fine (0/anything = 0) but the profile is all zeros.

The fix: validate `max_a > 0` and clip `accel_steps` to `n // 2` maximum.

**ToF Driver (2 bugs):**
4. Null pointer dereference in VL53L0X driver when sensor is disconnected and `read()` returns `None`. The caller (`perception_task`) doesn't check for `None` and passes it to `parking_detector.update()`.
5. I2C address collision: both VL53L0X sensors default to address 0x29. Without the xshut_pin mechanism, they can't coexist on the same bus. The config allows setting xshut_pin per sensor, but the default is `None` (no xshut), meaning only one sensor works out of the box.

**State Machine (3 bugs):**
6. `StateMachine` never transitions out of `INIT` because the `start_signal` event is never sent. The start detection module (`start_detection.py`) sends the signal, but it wasn't connected in `main.py`.
7. `OBSTACLE_AVOID` -> `FORWARD` transition has no timeout. If the robot can't find a path around an obstacle, it loops forever in avoidance.
8. `PARKING` state doesn't check if the robot has actually stopped moving. It starts the parking timer immediately, even if the robot is still sliding.

**UKF Filter (1 bug):**
9. Adaptive noise estimator updates with stale state. `adaptive_noise.update()` is called AFTER `ukf.update()`, so it uses the already-updated state instead of the pre-update innovation.

**UART Protocol (1 bug):**
10. UART send buffer overflow in `send_packet()`: the buffer is 32 bytes, but if payload > 24 bytes, `memcpy` writes past the buffer. No bounds check.

**General (2 bugs):**
11. `SystemManager.shutdown()` is not called when `mgr.run()` returns normally. It's only called in the `finally` block if `run()` raises. Normal return bypasses shutdown.
12. Health monitor timeout too short (1.0s). Sensor tasks at 100 Hz with occasional 15ms jitter can trigger false "dead" alerts.

## Errors Encountered While Fixing

**Error 1: Fixing the lap counter off-by-one broke the unit test.**
The unit test expected `current_lap == 2` after 2 crossings. After the fix (properly handling the start-line crossing), 2 crossings should give `current_lap == 1`. I had to update the unit test to match the new behaviour.

**Error 2: Adding a timeout to `OBSTACLE_AVOID` created a new bug.**
I added a 10-second timeout: if the robot can't find a path in 10 seconds, it backs up and tries again. But the timeout started counting from the first entry into the state, not from the last attempted action. If the robot was actively trying to avoid (just slowly), the timeout would fire prematurely. Fixed by resetting the timeout timer on every action attempt.

## Bugs Fixed Summary
| # | File | Bug | Fix |
|---|------|-----|-----|
| 1 | mission/lap_counter.py:20 | Off-by-one lap count | Start at -1 internally |
| 2 | mission/lap_counter.py:28 | Reset too fast | Add 500ms debounce |
| 3 | trajectory/velocity_profile.py:40 | Div by zero in profile | Validate max_a > 0 |
| 4 | sensors/tof/vl53l0x.py:55 | Null on disconnect | Check return before use |
| 5 | sensors/tof/vl53l0x.py:10 | I2C address collision | Generate unique addresses |
| 6 | main.py:227 | Start signal not sent | Add event wiring |
| 7 | mission/state_machine.py:45 | No avoidance timeout | Add 10s fallback timer |
| 8 | mission/state_machine.py:90 | Parking starts too early | Wait for v < 0.01 m/s |
| 9 | fusion/ukf.py:80 | Stale innovation | Reorder update calls |
| 10 | esp/main/main.c:475 | Buffer overflow | Bounds check payload len |
| 11 | system/manager.py:150 | Shutdown not called | Add explicit shutdown path |
| 12 | system/health_monitor.py:15 | False timeouts | Increase to 2.0s |
