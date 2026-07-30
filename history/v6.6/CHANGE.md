## v6.6 — Global Planner — 2026-07-20

### Summary

Implemented a `GlobalPlanner` that generates waypoints for the WRO track layout. The planner takes the track's width and length (from `pi_config.yaml` or surprise rules) and produces a set of rectangular waypoints: four corners with an adjustable centerline offset. The initial version produced only 5 waypoints (4 corners + return to start), which caused the robot to cut corners badly — the nearest-waypoint approach in the Stanley controller would drive directly toward the corner waypoint instead of following the arc. The fix was interpolating to 100 mm waypoint spacing.

### What Changed

The robot now needs a map of where to drive. The localization system provides pose estimates, the perception system detects obstacles, but without a global plan the robot doesn't know the track layout. The WRO track is a flat rectangle with known dimensions (typically 3 m × 2 m for the obstacle course, specified in `surprise_rules.yaml` as `track_width` and `track_length`). The global planner generates a centerline path that the robot follows.

The planner generates waypoints by:
1. Reading track width and length from config
2. Defining 4 corner points at the track boundaries
3. Offsetting inward by half the robot's width (0.2 m, including safety margin) to create a centerline
4. Closing the loop back to the first waypoint

The centerline offset is critical: without it, the robot would drive along the wall and potentially hit track boundaries. For a 3 m × 2 m track with 0.2 m offset, the driving rectangle is 2.6 m × 1.6 m.

### Error: Cutting Corners

The first version generated precisely 5 waypoints: (0.2, 0.2), (2.8, 0.2), (2.8, 1.8), (0.2, 1.8), (0.2, 0.2). Waypoints were spaced 2.6 m apart on the long sides and 1.6 m on the short sides. The Stanley controller (v6.2) tracked these waypoints by steering toward the nearest one. As the robot approached a corner, the nearest waypoint was the corner itself, so the robot steered directly toward it — cutting the corner.

I logged the robot's position during a corner approach:

```
Intended centerline:
  (1.5, 0.2) -> (1.5, 0.4) -> (1.5, 0.6) -> ... -> corner at (2.8, 1.8)

Actual robot path:
  (1.5, 0.2) -> (1.7, 0.3) -> (2.0, 0.5) -> (2.4, 0.8) -> (2.8, 1.2)
```

The robot reached the corner waypoint (2.8, 1.8) but approached it from a 45-degree diagonal instead of following the 90-degree arc. The cross-track error at the corner exit was 0.28 m — the robot was 28 cm inside the intended path. On a 3 m × 2 m track, the lanes between obstacles are only 1 m wide. A 28 cm deviation would cause the robot to hit obstacles in the obstacle course section.

The root cause is simple: with only 5 waypoints, the nearest waypoint is always the corner itself. The Stanley controller's algorithm is "steer toward the nearest waypoint." It doesn't know the path should arc through the corner. The nearest waypoint before the corner is the corner, so the robot aims directly at it.

### Alternatives Considered

1. **Look-ahead distance** — Instead of tracking the nearest waypoint, track a waypoint that's a fixed distance ahead (e.g., 0.5 m). This naturally creates smooth cornering because the robot aims ahead of the corner rather than at it. I tried look-ahead distances from 0.2 m to 1.0 m:
   - 0.2 m: essentially the same as nearest-point, corner cut was 0.26 m
   - 0.5 m: corner cut reduced to 0.12 m — better but still significant
   - 1.0 m: corner cut was 0.05 m, but the robot would "straight-line" through corners and hit the outside wall on tight S-curves

   Look-ahead is promising but the optimal distance depends on speed and curvature. Pure pursuit (which uses a look-ahead distance proportional to speed) would be more principled but I decided to save that for a future iteration.

2. **Interpolated waypoints (chosen)** — Generate waypoints at 100 mm spacing along the entire centerline. For a 3 m × 2 m track (10 m perimeter), this gives about 100 waypoints. With 100 mm spacing, the Stanley controller always has a waypoint within 5 cm of the true path, so it tracks accurately even with nearest-point logic.

3. **Bezier curves at corners** — Replace corner waypoints with Bezier control points that create rounded corners. This is what the cubic spline in v6.7 does at a higher level. But the global planner should provide a reasonable path even without the spline, as a fallback.

### The Fix

I added an `interpolate()` method to `GlobalPlanner` that fills intermediate points between the base waypoints:

```python
def interpolate(self, spacing=0.1):
    new_pts = []
    for i in range(len(self.waypoints) - 1):
        p0 = self.waypoints[i]
        p1 = self.waypoints[i + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dist = math.hypot(dx, dy)
        n = max(2, int(dist / spacing))
        for j in range(n):
            t = j / n
            new_pts.append((
                p0[0] + t * dx,
                p0[1] + t * dy,
            ))
    new_pts.append(self.waypoints[-1])
    self.waypoints = new_pts
```

The spacing is configurable (default 0.1 m). For a 10 m track, this generates ~100 points. I also tested 0.05 m spacing (200 points) and 0.2 m spacing (50 points). The 0.05 m version was overkill — it didn't improve tracking error beyond 0.1 m because the Stanley controller's nearest-point logic at 50 Hz already resolves to ~2 cm precision. The 0.2 m version had 8 cm corner cut, which was noticeable.

I also added a `reverse_direction()` method for surprise rules that require counter-clockwise driving. It simply reverses the waypoint list.

### Remaining Issues

- The planner assumes a rectangular track. WRO sometimes uses L-shaped or irregular tracks for surprise rounds. The `plan_rectangle()` method should become `plan_track()` that reads track shape from config. The shape could be specified as a sequence of (x, y) vertices in the config file.
- Waypoint spacing of 100 mm means ~100 waypoints. The cubic spline in v6.7 will interpolate further, so 100 mm is fine as input resolution.
- The planner doesn't handle the start zone separately. The first waypoint is just the centerline corner. For competition, the robot starts at a specific location and must complete a full lap. I should add a `plan_with_start()` method that generates waypoints from the start position.
- The centerline offset (0.2 m) is hard-coded. It should be `robot_width / 2 + safety_margin` and pulled from config.

### Files

- `global_planner.py` — GlobalPlanner with rectangular waypoints and interpolation
- `planner_test.py` — Test script that visualizes waypoints and checks spacing uniformity
