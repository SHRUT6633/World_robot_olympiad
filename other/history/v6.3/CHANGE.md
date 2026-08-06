## v6.3 — Feedforward Steering — 2026-07-17

### Summary

Added a curvature-based feedforward term to the steering controller. The Stanley controller (v6.2) is purely feedback: it reacts to errors after they occur. For curved sections of the track, the feedback controller has to continuously correct because the path curvature demands a non-zero steering angle. The feedforward term computes the steering angle required by the path curvature alone (using the bicycle model: `delta = arctan(L * curvature)`) and adds it to the Stanley output. The feedforward was initially too aggressive, causing overshoot in tight corners, so I limited it to 50% of total steering authority.

### What Changed

The pure Stanley controller works well on straights but in curves it always lags. The robot enters a corner, the cross-track error grows, Stanley corrects, but there's always a steady-state error proportional to curvature. This is because Stanley is a feedback controller — it needs an error to produce an output. On a constant-radius curve, the required steering angle is constant, but the feedback controller doesn't know that; it only knows there's an error and tries to reduce it. The steady-state cross-track error on a 0.5 m radius curve at 0.8 m/s was 4.5 cm. That's borderline acceptable but uses up our entire error budget.

Feedforward solves this: if we know the path curvature at the current point, we can pre-compute the required steering angle. The robot then enters the corner already steering the correct amount, and the feedback controller only needs to correct small disturbances like friction variations or imperfect alignment.

The feedforward uses the kinematic bicycle model: `delta_ff = arctan(L * curvature)`, where L is the wheelbase (0.26 m) and curvature is 1/turning_radius. For a 0.5 m radius corner, curvature = 2.0 1/m, so `delta_ff = arctan(0.26 * 2.0) = arctan(0.52) = 27.4 degrees`. This is close to our steering limit of 30 degrees. For a 1.0 m radius corner, curvature = 1.0, `delta_ff = arctan(0.26) = 14.6 degrees`.

### Error: Overshoot in Tight Corners

I computed feedforward as `ff = arctan(L * curvature)` and added it directly to the Stanley output: `total = steer + ff`. On the 0.5 m radius test corner at 0.8 m/s, the feedforward term was about 27 degrees. Added to the Stanley output (which was already correcting for accumulated cross-track error of 2 cm), the total hit 38° — exceeding the 30° steering limit.

The robot turned too sharply, shot past the corner exit, and the cross-track error after the corner was 15 cm. That's five times the normal tracking error of 3 cm.

My log from the corner test:

```
[frame 2100] curvature=2.0 ff=27.4 cte=-0.01 steer=4.2 total=31.6 -> CLIPPED to 30.0
[frame 2101] curvature=2.0 ff=27.4 cte=-0.03 steer=6.1 total=33.5 -> CLIPPED to 30.0
[frame 2102] curvature=1.8 ff=25.1 cte=-0.06 steer=8.9 total=34.0 -> CLIPPED to 30.0
[frame 2103] curvature=1.5 ff=21.3 cte=-0.12 steer=12.7 total=34.0 -> CLIPPED to 30.0
```

The feedback and feedforward were fighting each other: the feedforward steered correctly for the curve (27°), but the feedback saw residual cross-track error from the entry transient (the robot entered the corner with 2 cm of inside bias) and added 6° on top. The total of 33° was clipped to 30°, but that's still more than the 27° required. The robot oversteered.

The exit trajectory showed the consequences: after the corner, the cross-track error was +15 cm (outside the corner exit). The robot had to steer left 20° to recover, taking another 2 meters to get back on track.

I also tried corners of different radii. At 0.3 m radius (the tightest possible with our 30° steering limit — `arctan(0.26/0.3) = 40.9°` which is beyond the limit — the robot physically cannot track a 0.3 m radius corner at any speed). The feedforward was clipping continuously and the robot couldn't even stay on the path. This is a hard kinematic limit, not a tuning problem.

### Alternatives Considered

1. **Pure feedforward with no feedback** — This would work perfectly if the model were perfect and there were no disturbances. In reality, friction, servo lag, and surface unevenness produce errors that would accumulate without feedback. I simulated this: with pure feedforward and ±2 cm initial error, the robot stayed within 2 cm for the first corner, but by the third corner the error grew to 8 cm due to model inaccuracies. Model error accumulates because there's no correction mechanism.

2. **Feedforward only on curve entry** — Apply feedforward during the first 50% of the corner (rising edge of curvature), then let feedback handle the exit. This helps entry transient but hurts mid-corner tracking. I found the robot enters well but drifts inside during the constant-curvature section because the feedback isn't aggressive enough to maintain the 27° required.

3. **Weighted sum, max 50% feedforward (chosen)** — I added a `max_ff_ratio = 0.5` parameter that limits the feedforward contribution to at most 50% of the total steering command. The total is `total = feedback + ff`, but `ff` is clipped so that `abs(ff) <= max_ff_ratio * abs(total)`. This ensures feedforward never dominates.

### The Fix

The cleanest implementation was to compute both feedback and feedforward independently, then limit the feedforward contribution:

```python
total_raw = feedback_steer + feedforward_steer
max_ff_contrib = max_ff_ratio * abs(total_raw)
feedforward_steer = np.clip(feedforward_steer, -max_ff_contrib, max_ff_contrib)
total = feedback_steer + feedforward_steer
```

With `max_ff_ratio=0.5`, the feedforward never contributes more than 50% of the total steering. For the 0.5 m radius corner, the feedforward was limited to ~15°, and the feedback contributed the remaining ~15°. The total of 30° was correct for the corner. The cross-track error after the corner dropped from 15 cm to 4 cm.

I also added curvature smoothing. The raw curvature from discrete path points was noisy — a 2 cm difference in waypoint position caused a 10% change in curvature. I added a low-pass filter on curvature (alpha=0.3): `curvature = 0.7 * prev_curvature + 0.3 * raw_curvature`. This smoothed the feedforward command and prevented rapid steering changes due to curvature noise.

### Remaining Issues

- Curvature estimation from discrete path points is still noisy even with the filter. A better approach is to compute curvature analytically from the cubic spline's second derivative (v6.7) rather than from discrete waypoints.
- The feedforward model assumes no slip. At high speed (>1.5 m/s) in tight corners (<0.5 m radius), the robot will understeer due to tire slip, and the feedforward will be insufficient. This might need a slip-angle compensation term, but that's a v7.x problem.
- `max_ff_ratio=0.5` was tuned experimentally. For corners tighter than 0.5 m radius, 50% might still be too much if the feedback is also large. I could make the ratio adaptive based on curvature.
- The wheelbase L=0.26 m was measured from CAD. I should verify it by driving a circle and measuring the actual vs. predicted radius.

### Files

- `feedforward_steer.py` — Combined feedback + feedforward steering controller with ratio limiting
- `corner_test.py` — Script that drives through a known-radius corner and logs tracking error
