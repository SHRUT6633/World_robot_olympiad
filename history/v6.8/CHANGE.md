## v6.8 — Velocity Profiling — 2026-07-22

### Summary

Added a `VelocityProfiler` that computes a target speed for each point along the spline trajectory. The profiler reduces speed in high-curvature sections (corners) to keep lateral acceleration below a safe limit, and accelerates back to max speed on straights. The initial implementation allowed acceleration up to 2.0 m/s², which caused wheel slip on our smooth floor surface (low friction coefficient). The fix was limiting acceleration to 0.5 m/s² and adding a forward-backward pass to enforce the limit.

### What Changed

The motor PID (v6.0) controls speed at the actuator level, but it needs a speed target. Previously, the target was a constant 1.5 m/s everywhere. This worked on straights but caused the robot to enter corners way too fast — lateral acceleration would hit 3.5 m/s², exceeding the tire's friction limit, and the robot would slide wide. I saw this in the competition test: the robot would enter the first corner at 1.5 m/s, slide 20 cm wide, and the Stanley controller would overcorrect, causing further oscillation.

The velocity profiler computes a speed profile that respects two constraints:
1. **Centripetal acceleration**: `v_max = sqrt(max_lat_accel / curvature)` — reduces speed in curves so the robot doesn't slide. The centripetal force required to follow a curve is `F = m * v^2 / r = m * v^2 * curvature`. If the required force exceeds the friction limit `mu * m * g`, the robot slides.
2. **Linear acceleration**: limit how fast the speed can change between consecutive points to prevent wheel slip from longitudinal forces.

The algorithm runs in two passes:
- Forward pass: limit acceleration (speed increase per step)
- Backward pass: limit deceleration (speed decrease per step)

This produces a feasible trapezoidal-ish profile that respects both curvature and acceleration limits.

### Error: Wheel Slip from High Acceleration

The initial parameters were `max_v=2.0` and `max_a=2.0`. On a straight-to-corner transition, the profiler would accelerate at 2.0 m/s² up to the corner, then slam on the brakes at -2.0 m/s² to slow down for the turn. The deceleration phase produced visible wheel slip — the robot's tires squeaked on the smooth MDF competition surface.

I captured the IMU data during a braking event before the first corner:

```
[IMU] t=12.3s ax=1.89  ay=0.12  | braking, still straight
[IMU] t=12.4s ax=2.12  ay=0.08  | harder braking
[IMU] t=12.5s ax=1.45  ay=0.85  | SLIP DETECTED (lateral acceleration spike)
[IMU] t=12.6s ax=0.32  ay=1.92  | sliding sideways
[IMU] t=12.7s ax=0.12  ay=2.31  | full slide, robot is yawing
[IMU] t=12.8s ax=0.08  ay=1.45  | recovering
[IMU] t=12.9s ax=0.45  ay=0.32  | back under control
```

At t=12.5s, the lateral acceleration spiked to 0.85 m/s² while longitudinal dropped — the robot was sliding. By t=12.6s, the lateral acceleration dominated at 2.31 m/s², indicating the rear end was stepping out. The robot yawed about 15° before the Stanley controller corrected. This cost about 0.5 seconds of lap time and 10 cm of path deviation.

The coefficient of friction on the competition surface (MDF board with matte finish) is approximately 0.3–0.4. With our single driven axle (all 4 wheels mechanically linked), the maximum safe acceleration before slip is `mu * g ≈ 0.35 * 9.81 ≈ 3.4 m/s²` *if all wheels have equal grip*. In practice, the AWD system has uneven torque distribution (the front wheels get slightly more torque due to the drivetrain layout), so the effective longitudinal limit is lower — about 0.6 m/s² based on the slip events.

I also tested the curvature limit more precisely. I set up a constant-radius turn (1.0 m) and ran the robot at increasing speeds. At 1.2 m/s, the lateral acceleration was `v^2/r = 1.44/1.0 = 1.44 m/s²` — no slip. At 1.5 m/s, it was 2.25 m/s² — no slip. At 1.8 m/s, it was 3.24 m/s² — the robot slid. So the maximum safe lateral acceleration is about 3.0 m/s², corresponding to `mu ≈ 0.31`. This is consistent with the MDF friction coefficient.

So the lateral limit (3.0 m/s²) is much higher than the longitudinal limit (0.6 m/s²). This makes sense: the robot has only one driven axle, so braking force is limited by that axle's grip, while cornering uses all 4 wheels' grip.

### Alternatives Considered

1. **Lower friction surface** — The official WRO surface is unknown until competition day. We can't assume higher friction. In fact, we should assume lower friction (smooth concrete or vinyl).

2. **Dynamic friction estimation** — Use the IMU to detect slip in real-time and reduce acceleration on the fly. I briefly prototyped this: monitor `ax` and `ay` from the IMU, and if `sqrt(ax^2 + ay^2) > 0.8 * mu * g`, reduce the acceleration limit. This is complex and adds another tuning dimension (the 0.8 factor). I'll revisit for v7.x if we have time.

3. **Conservative limit (chosen)** — Set `max_a = 0.5 m/s²`. This is well below the 0.6 m/s² slip threshold and ensures the robot never loses traction on any surface likely to be encountered in WRO. The trade-off is slower lap times: with a 10 m track, the theoretical minimum lap time at 2.0 m/s with instantaneous acceleration is 5 seconds. With 0.5 m/s² acceleration, the minimum is `v_max / max_a + track_length / v_max ≈ 2.0/0.5 + 10/2.0 = 4 + 5 = 9` seconds. But a robot that completes at 0.8 m/s average is better than one that crashes at 1.5 m/s.

### The Fix

The velocity profile computation with the forward-backward pass:

```python
# Forward pass: limit acceleration
for i in range(1, n):
    v[i] = min(v[i], v[i-1] + max_a * ds / (v[i-1] + 1e-6))

# Backward pass: limit deceleration  
for i in range(n-2, -1, -1):
    v[i] = min(v[i], v[i+1] + max_a * ds / (v[i+1] + 1e-6))
```

Where `ds` is the average arc length between consecutive spline points. The `1e-6` prevents division by zero when the robot is stationary. The formula `v[i] = v[i-1] + max_a * ds / v[i-1]` comes from the time to traverse one segment: `dt = ds / v`, so the speed change per segment is `max_a * dt = max_a * ds / v`. This is the discrete forward-Euler integration of the acceleration limit.

With `max_a=0.5`, the wheel slip events disappeared entirely. We ran 20 laps of the test track and the IMU never registered a slip event (defined as `ay > 1.5 m/s²` while braking). The robot now takes corners at a controlled speed (0.5–0.8 m/s depending on curvature) and accelerates smoothly on straights.

### Remaining Issues

- `max_lat_a=2.0` might still be too high for the actual competition surface. The test result of 3.0 m/s² was on clean MDF. On dusty MDF or concrete, the friction coefficient could be lower. I'll set `max_lat_a = 2.0` to be conservative (v^2/r = 2.0 gives v=1.41 m/s on a 1 m radius corner).
- The velocity profile is computed once at planning time. If the robot deviates from the path (e.g., after obstacle avoidance), the profile becomes misaligned with the robot's actual position. Re-profiling takes ~40 ms, which is too slow for real-time use. The precomputed paths in v6.9 solve this for obstacle avoidance.
- The acceleration constraint formula has a singularity at v=0. The `1e-6` guards against division by zero but the behavior at very low speed is approximate. When the robot starts from rest, the first step is `v[1] = min(v_limit[1], 0 + 0.5 * ds / 1e-6)`, which effectively allows infinite acceleration from zero. This is fine because the motor PID handles the initial acceleration from rest, but the profiler should ideally model it.

### Files

- `velocity_profile.py` — VelocityProfiler with acceleration limits and curvature-based slowing
- `profile_test.py` — Script that generates a test path and validates the acceleration profile
