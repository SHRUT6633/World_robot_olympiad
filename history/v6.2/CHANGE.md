## v6.2 — Stanley Steering Controller — 2026-07-16

### Summary

Implemented the Stanley lateral controller for path tracking. The controller computes a steering angle from two terms: heading error (difference between robot heading and path heading) and cross-track error (lateral distance to the nearest path point). The Stanley law is `steer = heading_error + arctan(k * crosstrack / (k_soft + v))`. The initial version used a fixed gain k=1.0, which caused the robot to oscillate around the path at low speed. The fix was making the gain speed-dependent.

### What Changed

The robot now has a meaningful path to follow. The global planner (v6.6, later) generates waypoints, and the cubic spline trajectory (v6.7) creates a smooth dense path. For now, the test path is a simple rectangle with interpolated waypoints at 10 cm spacing. The Stanley controller is the lateral control layer that runs at 50 Hz: at each tick it finds the nearest point on the reference path, computes heading error and cross-track error, then outputs a steering angle in radians.

The steering angle feeds into `servo_pid.py` (v6.1) which handles the low-level servo positioning. The separation of concerns is clean: Stanley worries about driving the path, ServoPID worries about hitting the commanded angle. The two run at different rates: Stanley at 50 Hz, ServoPID at 100 Hz. The ServoPID's faster rate ensures the servo is always at the commanded position when Stanley ticks.

I also compute the target heading from the direction of the path at the nearest point. I look at the next waypoint after the nearest one to get the local path direction. This is important: if I used the heading of the nearest waypoint itself, the heading would be quantized to the waypoint spacing (which is 10 cm, so at 1 m/s the heading updates every 100 ms). Looking ahead one waypoint smooths the heading reference.

### Error: Oscillation at Low Speed

On the straights at 1.5 m/s, the robot tracked beautifully — cross-track error stayed under 3 cm. But when it entered a corner and the speed dropped to 0.4 m/s (from the velocity profile I was testing), the steering started oscillating. The robot would overcorrect left, then overcorrect right, with increasing amplitude until the Stanley output hit the ±30° steering limit.

I added path logging to trace the issue:

```
[frame 1420] speed=0.42 cte=0.023 he=0.12 steer=0.18  <- looks fine
[frame 1421] speed=0.41 cte=0.018 he=0.14 steer=0.19
[frame 1422] speed=0.40 cte=0.009 he=0.16 steer=0.20
[frame 1423] speed=0.39 cte=-0.004 he=0.18 steer=0.19  <- crossing over
[frame 1424] speed=0.38 cte=-0.021 he=0.20 steer=0.14  <- correction
[frame 1425] speed=0.38 cte=-0.042 he=0.22 steer=0.06  <- overcorrecting
[frame 1426] speed=0.37 cte=-0.065 he=0.24 steer=-0.02 <- wrong direction!
```

The oscillation period was about 20 frames = 400 ms = 2.5 Hz. The wavelength at 0.4 m/s is 0.16 m. So the robot was weaving left-right every 16 cm along the path. I could see this in the wheel tracks on the test mat — a visible sinusoidal pattern.

The problem is in the denominator of the Stanley law: `k_soft + v`. At v=0.4 m/s with k_soft=1.0, the effective cross-track gain is `k / (1.0 + 0.4) = 0.71 * k`. At v=1.5 m/s it's `k / (1.0 + 1.5) = 0.4 * k`. So at low speed the cross-track correction is actually *stronger*, not weaker. This makes the robot more aggressive at low speed when it should be more cautious.

The core issue: the `k` gain was tuned for 1.5 m/s driving. At low speed, the robot's dynamics change — the steering response is more immediate (less momentum smoothing the corrections), and the same k value causes overcorrection. The Stanley law's `v` in the denominator is meant to reduce correction at high speed (where the robot covers more ground per correction), but it has the inverse effect of increasing correction at low speed.

I confirmed this by simulating the closed-loop response in a Python script. With k=1.0 and v=0.4, the closed-loop poles were at 0.85 ± 0.3j (damping ratio ζ=0.28, underdamped). At v=1.5, the poles were at 0.6 ± 0.15j (ζ=0.62, well-damped). The low-speed underdamping caused the oscillation.

### Alternatives Considered

1. **Increase k_soft** — A larger k_soft (e.g., 5.0) would reduce the cross-track gain at all speeds and especially at low speed. But it also reduces the maximum cross-track correction, making the robot sluggish on straights at high speed. With k_soft=5.0, the cross-track correction at v=0.4 is k/(5.4) = 0.185*k, and at v=1.5 it's k/(6.5) = 0.154*k. The ratio between them is 1.2:1 instead of 1.75:1, so the speed dependency is reduced. But the maximum steering correction from cross-track error drops from ~26° to ~11° at v=0.4, which is too weak for tight corners.

2. **Pure pursuit controller** — Pure pursuit uses a look-ahead distance instead of cross-track error. It's more stable at low speed because the look-ahead distance naturally reduces cornering aggressiveness as speed drops. But it has higher tracking error at high speed (the robot always lags the path by the look-ahead distance). I considered switching entirely but decided Stanley is more appropriate for WRO because the tracking error is bounded at high speed, which matters for the narrow track sections.

3. **Speed-dependent k (chosen)** — Make the gain k a function of speed: k = 1.0 at v >= 1.0 m/s, k = 0.5 at v <= 0.3 m/s, linear interpolation in between. This is essentially gain scheduling for the lateral controller, not unlike what I did for the motor PID in v6.0.

### The Fix

I added `_select_k(v)` to StanleyController:

```python
def _select_k(self, v):
    if v >= 1.0:
        return 1.0
    elif v <= 0.3:
        return 0.5
    else:
        t = (v - 0.3) / 0.7
        return 0.5 + t * 0.5
```

This interpolates k linearly between 0.5 at 0.3 m/s and 1.0 at 1.0 m/s. Below 0.3 m/s it floors at 0.5. The oscillation disappeared. Cross-track error on the test rectangle stayed under 5 cm at all speeds. The 2.5 Hz weave was gone.

I also noticed that the heading error term doesn't need speed-dependent scaling — the `arctan` term is the problematic one. The heading error directly contributes the correct steering direction regardless of speed. I verified this by running the controller with heading_error only (no cross-track term) at both high and low speeds — the heading error response was stable in both cases. So the fix only needed to scale the cross-track gain.

### Remaining Issues

- The controller assumes the nearest path point is the correct target. On tight switchbacks or S-curves, the nearest point might be behind the robot, causing reverse tracking. This needs look-ahead logic (pick the point that's a fixed distance ahead, not the absolute nearest).
- k_soft=1.0 was a guess from the Stanford paper (they used k_soft=1.0 for the DARPA Grand Challenge). Our robot is much smaller and has different dynamics. I should characterize the optimal k_soft experimentally.
- Stanley only handles lateral control. Longitudinal control (speed) is handled separately by the velocity profiler (v6.8), and the two can interfere — a Stanley correction that changes the steering angle affects the lateral acceleration, which should ideally be accounted for in the speed profile.

### Files

- `stanley.py` — Stanley lateral controller with speed-dependent gain `_select_k()`
- `stanley_tune.py` — Tuning script that sweeps k values and measures cross-track error
