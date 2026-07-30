## v6.4 — Gain Scheduling — 2026-07-18

### Summary

Formalized a gain scheduling framework that switches PID gains based on the robot's speed range. The original motor PID (v6.0) used a hard-coded gain map in `_select_gains()`. For v6.4, I extracted this into a reusable `GainScheduler` class that can be used by any controller (motor PID, servo PID, Stanley gain). The scheduler defines three speed zones (slow: 0–0.5 m/s, medium: 0.5–1.0 m/s, fast: 1.0–2.0 m/s) and interpolates gains linearly between zones to eliminate the jerk caused by abrupt gain changes.

### What Changed

The gain scheduling logic was embedded inside `MotorPID._select_gains()` with hard-coded thresholds and gains. This worked for the motor but wasn't reusable for the servo controller (v6.1) or the Stanley controller (v6.2). Both of those would benefit from speed-dependent gains too — the servo needs higher gains at low speed for precise parking maneuvers, and Stanley needs lower gains at low speed to prevent oscillation (which I already added ad-hoc in v6.2).

I factored the scheduling logic into `GainScheduler` that stores a matrix of gains indexed by zone and provides a `select(v)` method that returns interpolated gains for any speed. The scheduler holds three gain presets:
- **slow** (0–0.5 m/s): high kp=0.8, ki=0.05, kd=0.02 — aggressive control for precise low-speed maneuvering
- **medium** (0.5–1.0 m/s): medium kp=0.5, ki=0.03, kd=0.01
- **fast** (1.0–2.0 m/s): low kp=0.3, ki=0.01, kd=0.005 — more conservative to avoid oscillation at speed

The gains decrease with speed because at higher speeds, the robot's inertia provides "natural damping" and aggressive control inputs would cause oscillation. At low speeds, we need more control authority to overcome static friction and achieve precise positioning.

### Error: Jerk from Abrupt Gain Changes

The original `_select_gains()` used if-else with fixed thresholds. When the robot accelerated through 0.5 m/s while in a corner, the kp would jump from 0.8 to 0.5 in a single control tick. The sudden reduction in proportional gain caused the motor PID output to drop abruptly. The robot would lurch — a visible and audible jerk that the IMU registered as a 2.3 m/s² spike.

I caught this during a test run with the IMU logging:

```
[frame 4300] speed=0.49 zone=slow kp=0.8 steer=0.15 accel_axial=0.12
[frame 4301] speed=0.51 zone=medium kp=0.5 steer=0.09 accel_axial=2.31 <- JERK
[frame 4302] speed=0.53 zone=medium kp=0.5 steer=0.08 accel_axial=0.45
[frame 4303] speed=0.55 zone=medium kp=0.5 steer=0.08 accel_axial=0.18
```

The steering angle dropped by 40% (from 0.15 rad to 0.09 rad) in 10 ms. The robot's inertia caused a sudden lateral jerk of 2.31 m/s². This is both mechanically stressful (it could strip the servo gears over time) and bad for tracking accuracy — the robot deviated from the path by 2.3 cm in that single tick.

The same problem occurred at the 1.0 m/s threshold going from medium to fast gains, though the jerk was smaller (1.1 m/s²) because the gain change was smaller (kp from 0.5 to 0.3).

I instrumented the code with a running log of gain changes. Over a 60-second test run with speed varying from 0 to 2.0 m/s, there were 47 gain transitions. Each one produced a jerk spike. The cumulative effect was a shaky ride that would cost points in the "smooth driving" criterion.

### Alternatives Considered

1. **Hysteresis** — Use different thresholds for increasing vs decreasing speed (e.g., switch to medium at 0.55 m/s when accelerating, but stay in medium until 0.45 m/s when decelerating). This prevents rapid switching at the boundary (chattering) but doesn't eliminate the step change itself. The jerk would still happen, just less frequently. I tested this with 0.05 m/s hysteresis and the number of transitions dropped from 47 to 31, but each one still had a 2+ m/s² jerk.

2. **Fuzzy logic** — A full fuzzy controller with membership functions could smooth the transition, but it's overkill for three zones and adds significant complexity. I'd need to define membership functions, rule bases, and defuzzification. The implementation time would be 2-3 days.

3. **Linear interpolation between zones (chosen)** — Instead of a step change, the scheduler interpolates each gain linearly across a 0.1 m/s transition band around the threshold. Within [0.45, 0.55] m/s, the gains blend smoothly from the slow preset to the medium preset. This is simple to implement and requires no tuning parameters beyond the transition width.

### The Fix

I added an interpolation method to `GainScheduler`:

```python
def _interpolate(self, v, low_speed, high_speed, low_gains, high_gains):
    t = (v - low_speed) / (high_speed - low_speed)
    t = max(0.0, min(1.0, t))
    return {
        k: low_gains[k] + t * (high_gains[k] - low_gains[k])
        for k in low_gains
    }
```

The transition band is 0.1 m/s wide centered on each threshold. For the 0.5 m/s threshold, the band is [0.45, 0.55] m/s. Within this band, kp transitions linearly from 0.8 to 0.5, ki from 0.05 to 0.03, kd from 0.02 to 0.01.

This eliminated the jerk entirely. The IMU showed a smooth 0.3 m/s² transition over about 200 ms (20 control ticks at 100 Hz) instead of a 2.3 m/s² spike in 10 ms. The robot's trajectory was perfectly smooth through the transition.

I also made the scheduler configurable via a list of zone definitions, so the zones, gains, and transition bandwidth can be set in the YAML config file. This makes it possible to tune the robot's behavior for different track surfaces without code changes.

### Remaining Issues

- The scheduler uses the current speed to select gains. But the robot's response depends on both current speed AND target speed. During acceleration, the current speed lags the target speed, so the scheduler might select higher gains than appropriate for the actual state. For example, if the target jumps from 0 to 2.0 m/s, the current speed is 0, so the scheduler selects slow-zone gains. These are too aggressive for the initial acceleration (the motor saturates at 255 PWM anyway, so the gains don't matter much). But when the current speed reaches 0.5 m/s, the scheduler switches to medium-zone gains, which is happening while the robot is still accelerating hard.

- Three zones might not be enough. A more optimal schedule might have five zones or continuous gain adaptation. But three zones with interpolation works well for the WRO speed range (0–2.0 m/s). The maximum gain deviation from optimal is about 10% based on my characterization data.

- The transition bandwidth (0.1 m/s) was tuned empirically. Different surfaces or loads might need different bandwidths. On a high-friction surface (rubber mat), the jerk from a step change was smaller (0.8 m/s²), so a narrower band (0.05 m/s) would suffice. On a low-friction surface (smooth concrete), the jerk was larger (3.5 m/s²), requiring a wider band (0.15 m/s).

### Files

- `gain_schedule.py` — GainScheduler class with zone interpolation and configurable gains
- `scheduler_test.py` — Unit test that validates gain transitions across the full speed range
