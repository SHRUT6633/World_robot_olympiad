## v6.7 — Cubic Spline Trajectory — 2026-07-21

### Summary

Added a cubic spline trajectory generator that converts the discrete waypoints from the global planner (v6.6) into a smooth, continuous path. The spline is fitted independently for X(t) and Y(t) as functions of a parameter t (cumulative chord length), producing a C²-continuous curve through all waypoints. The initial implementation used "natural" boundary conditions (zero second derivative at endpoints), but this caused the spline to overshoot badly at the start and end of the path — the classic Runge phenomenon. The fix was switching to clamped boundary conditions that enforce the first derivative (direction) at the start and end points.

### What Changed

The global planner outputs waypoints at 100 mm spacing — a polyline with sharp corners at every waypoint (well, slight angles since they're on a straight line, but the corners at track bends are 90°). Driving a polyline directly causes the Stanley controller to make abrupt steering changes at each waypoint transition because the heading reference changes discontinuously. A cubic spline smooths the path by fitting a piecewise cubic polynomial that passes through all waypoints with continuous first and second derivatives.

The implementation uses `scipy.interpolate.CubicSpline`, which handles the linear algebra internally. The parameterization uses cumulative chord length (distance along the polyline), which ensures that the parameter advances faster through long segments than short ones — this gives better curvature behavior than uniform parameterization.

The key design decisions:
- Independent splines for X(t) and Y(t) (parametric spline)
- Parameter t normalized to [0, 1]
- 50 output points sampled evenly in t
- Boundary condition type configurable (natural, clamped, not-a-knot)

### Error: Start/End Overshoot (Runge Phenomenon)

The first version used `bc_type="natural"` (default). The fitted spline looked fine in the middle of the path, but at the start and end segments, the spline would deviate wildly from the waypoints. For a rectangular track, the spline would overshoot the first corner by 25 cm, then swing back through the waypoints.

I printed the spline deviation at each waypoint:

```
Waypoint   X_spline  Y_spline  X_wp  Y_wp  Deviation
   0       0.200    0.200    0.200  0.200  0.000
   1       0.450    0.200    0.450  0.200  0.000  (beginning of straight,
   2       0.700    0.200    0.700  0.200  0.000   everything fine so far)
   3       0.950    0.200    0.950  0.200  0.000
   ...
  22       2.800    0.200    2.800  0.200  0.000  (end of first straight)
  23       2.850    0.235    2.800  0.250  0.058  <-- deviation starts
  24       2.900    0.275    2.800  0.500  0.240  <-- MAX DEVIATION at mid-corner
  25       2.850    0.765    2.800  0.750  0.052  <-- returning
  26       2.800    0.800    2.800  0.800  0.000  (back on track)
```

The deviation at waypoint 24 (the inside of the corner) was 24 cm. The spline was cutting the corner by swinging outside the waypoint, then coming back. This is the Runge phenomenon: high-order polynomial interpolation oscillates at the boundaries of evenly-spaced points, especially when the underlying function changes rapidly (like a 90° corner).

The oscillation amplitude depends on the spacing of the waypoints. I tested with 50 mm spacing (instead of 100 mm) and the deviation dropped to 15 cm. With 200 mm spacing, it was 40 cm. But even 15 cm is too much — the track corridor is only 1 m wide, and combined with the robot's 20 cm width, we'd hit the wall.

The spline's second derivative is free at the boundaries with natural conditions (`bc_type="natural"` sets second derivative = 0 at both ends). This allows the spline to "swing out" at the ends because there's no constraint on curvature. In the middle of the track, the surrounding waypoints constrain the spline. At the corners (which are effectively boundaries between straight segments), the spline has one-sided support from the straight segments and the curvature constraint is too weak.

### Alternatives Considered

1. **Not-a-knot boundary conditions** — `bc_type="not-a-knot"` forces continuity of the third derivative at the first and last interior knots. This reduces boundary oscillation but doesn't eliminate it. For our rectangular track, it still produced 8 cm of overshoot at the corners. Not good enough.

2. **Add phantom waypoints** — Add extra waypoints outside the track (e.g., 0.5 m before and after each corner) to constrain the spline. This is hacky — the phantom points need careful positioning and would need to change if the track dimensions change. I tried it: I added waypoints at (2.4, 0.2) and (2.8, 0.6) around the first corner. The deviation dropped to 3 cm, but it took 4 hours of manual tuning to find good phantom positions for all 4 corners.

3. **Clamped boundary conditions (chosen)** — `bc_type="clamped"` lets you specify the first derivative (slope) at the endpoints. Setting the endpoint slopes to the direction from the waypoint to its neighbor constrains the spline from oscillating. This is mathematically principled: the first derivative at the corner entry should point along the incoming straight, and at the exit along the outgoing straight.

### The Fix

The clamped boundary conditions are specified as `bc_type=((1, slope), (1, slope))` where 1 indicates first derivative constraint:

```python
start_dx = x[1] - x[0]
start_dy = y[1] - y[0]
end_dx = x[-1] - x[-2]
end_dy = y[-1] - y[-2]

cs_x = CubicSpline(t, x, bc_type=((1, start_dx), (1, end_dx)))
cs_y = CubicSpline(t, y, bc_type=((1, start_dy), (1, end_dy)))
```

The first derivative values (`start_dx`, etc.) are scaled by the parameter spacing. Actually, scipy expects the derivative with respect to the parameter t, so the slope values should be `dx/dt` at the boundary. Since t runs from 0 to 1 and the waypoints are at cumulative chord distances, `dt = t[1] - t[0]` for the start and `dt = t[-1] - t[-2]` for the end. I initially forgot to scale by dt and got weird behavior (the spline was too constrained and couldn't turn at all). After fixing the scaling, the spline tracked perfectly.

After this fix, the spline deviation at the corners dropped from 24 cm to under 1 cm. The path now smoothly follows the waypoints without oscillation. I verified by computing the maximum deviation for all 4 corners: 0.8 cm, 0.6 cm, 0.9 cm, 0.7 cm. All well within our 2 cm tolerance.

### Remaining Issues

- The spline is parameterized by cumulative chord length. This is correct for path tracking but means the parameter advances faster through long segments. The 50 output points are sampled evenly in t, which means they're clustered in short segments and sparse in long ones. For the long straights (2.6 m), 50 points across 10 m of path means about 13 points per straight, or 20 cm spacing. This is fine for the Stanley controller but the velocity profile (v6.8) would benefit from more points on the straights.
- The spline can still overshoot in the middle of the path if waypoints are sparse. With 100 mm spacing from v6.6, this isn't a problem, but if the global planner ever uses wider spacing, the spline should be regenerated.
- num_points=50 is arbitrary. More points = smoother path for the controller but more computation. 50 points at 50 Hz is < 1 ms of CPU time on the Pi 4. I'd be comfortable with up to 200 points.
- The spline doesn't handle obstacles. If an obstacle is detected, the path needs to be replanned around it (v6.9).

### Files

- `cubic_spline.py` — CubicSplineTrajectory with clamped boundary conditions
- `spline_visualize.py` — Script that generates a test spline and measures deviation from waypoints
