## v6.9 — Obstacle Avoidance — 2026-07-23

### Summary

Added dynamic obstacle avoidance that replans the local path around detected obstacles. When the perception system detects an obstacle (pillar, wall, or surprise object), the avoidance system generates a deviation around it and feeds the modified path to the Stanley controller. The initial replanning approach computed a new cubic spline from scratch whenever an obstacle was detected, which took 200 ms — during which the robot at 1.0 m/s traveled 20 cm and could collide with the obstacle. The fix was precomputing 3 alternative paths (left offset, right offset, and emergency stop) and switching instantly on detection.

### What Changed

The robot now has obstacle detection via the ToF sensors (VL53L1X, 4 m range) and camera-based pillar detection (from the perception package). When an obstacle appears within the safety distance (0.5 m), the nominal path is no longer safe. The robot needs to deviate around the obstacle while staying on the track.

I implemented `DynamicObstacleAvoidance` that sits between the global planner / cubic spline and the Stanley controller. It takes the nominal path and obstacle positions (from the object detector), and produces a modified path that goes around the obstacle. The modification is done by computing a convex combination of the nominal waypoint and an avoidance vector away from the nearest obstacle.

The architecture is:
1. Perception system publishes obstacle positions at 20 Hz
2. DynamicObstacleAvoidance.select_path() is called at 50 Hz
3. It checks if any obstacle is within detection_radius (0.5 m) of the robot
4. If yes, it selects an alternative path from precomputed set
5. The selected path is fed to the Stanley controller

### Error: Replan Latency

The first version computed an entirely new cubic spline through the obstacle-free space. The algorithm was:
1. Detect obstacle (from perception, ~10 ms latency)
2. Generate 3 new waypoints around the obstacle (entry, apex, exit) based on obstacle position — took ~12 ms
3. Merge these waypoints with the remaining nominal waypoints — ~1 ms
4. Fit a cubic spline through the merged waypoints — ~145 ms (this was the bottleneck)
5. Compute a new velocity profile — ~38 ms
6. Hand over to Stanley controller — ~1 ms

Total: ~200 ms from detection to new path. At 1.0 m/s, the robot travels 20 cm during those 200 ms. Since the obstacle was detected at 0.5 m (the detection radius), the robot closed to 0.3 m before the new path was ready — dangerously close to the obstacle.

```
Event timeline:
t = 0.00s  Obstacle enters detection radius (0.50 m from robot)
t = 0.01s  Perception system detects obstacle
t = 0.02s  Obstacle position published
t = 0.03s  DynamicObstacleAvoidance receives obstacle
t = 0.04s  Replanning: generating waypoints...
t = 0.04s  Replanning: generating avoidance waypoints (12 ms)
t = 0.05s  Replanning: merging with nominal path (1 ms)
t = 0.05s  Replanning: fitting cubic spline (145 ms) ... 
t = 0.20s  Spline done! Computing velocity profile (38 ms)
t = 0.24s  Profile done! Path handover
t = 0.24s  Robot is now 0.26 m from obstacle  <- TOO CLOSE
```

If the obstacle is a pillar with 0.1 m radius, the clearance is `0.26 - 0.1 - 0.1 (robot half-width) = 0.06 m`. That's 6 cm of clearance — a collision risk. If the robot's path tracking has any error (which it always does), it will hit the pillar.

I profiled the cubic spline fitting specifically. The scipy CubicSpline calls LAPACK routines (`dgbsv` for banded matrix solving) that are optimized for large matrices but have overhead for small ones. Our splines have 10–30 waypoints (5 nominal + 3 avoidance + 2 extra). The overhead of calling through Python → C → LAPACK for a 30×30 matrix is about 145 ms. A manual Python implementation using `np.linalg.solve` for the tridiagonal system took 50 ms, still too slow. An incremental approach that updates only the affected segments would be faster but requires significant refactoring.

### Alternatives Considered

1. **Faster spline solver** — Implement a manual cubic spline solver using a tridiagonal matrix algorithm (Thomas algorithm) in pure Python. I prototyped this and got 50 ms — better but still not good enough. The robot travels 5 cm in those 50 ms, and combined with the other latencies, the total is still ~100 ms.

2. **Incremental spline update** — Only update the spline in the vicinity of the obstacle, keeping the rest unchanged. The cubic spline's global continuity requires that changing one segment affects all segments, but I could split the spline at the obstacle region, fit a local spline, and stitch it back with C¹ continuity. This would be ~20 ms for a 5-point local spline. But the stitching introduces discontinuities at the boundaries. Not worth the complexity.

3. **Precomputed alternative paths (chosen)** — Precompute 3 paths before the run, at the same time as the nominal path:
   - **PATH_A**: Nominal centerline (no deviation)
   - **PATH_B**: Offset 0.3 m to the left of nominal (for obstacles on the right)
   - **PATH_C**: Offset 0.3 m to the right of nominal (for obstacles on the left)
   
   When an obstacle is detected on the nominal path at close range, the system instantly switches to the appropriate precomputed path. No spline fitting or velocity profiling at runtime.

   The precomputation happens once during the planning phase (when the global planner is initialized), and the 3 paths are cached. The total precomputation time is ~300 ms for all 3 paths, but this is done before the robot starts moving.

### The Fix

I precompute the alternative paths in `precompute()`:

```python
def precompute(self, nominal_path):
    self.paths = {
        "center": np.array(nominal_path),
        "left": self._offset_path(np.array(nominal_path), self.offset),
        "right": self._offset_path(np.array(nominal_path), -self.offset),
        "brake": self._brake_path(np.array(nominal_path)),
    }
```

The `_offset_path` method shifts each point perpendicular to the path direction:

```python
def _offset_path(self, path, offset):
    result = np.copy(path)
    for i in range(1, len(path) - 1):
        dx = path[i+1, 0] - path[i-1, 0]
        dy = path[i+1, 1] - path[i-1, 1]
        norm = np.hypot(dx, dy) + 1e-6
        result[i, 0] += -dy / norm * offset
        result[i, 1] += dx / norm * offset
    return result
```

The offset is 0.3 m, which gives 0.2 m clearance on each side of the robot (robot width is 0.2 m, so 0.3 m offset means the robot center is 0.3 m from the nominal path, and the robot edge is 0.3 - 0.1 = 0.2 m from the nominal path). This is enough to clear a 0.1 m radius pillar with 0.1 m safety margin.

At runtime, `select_path()` chooses based on obstacle position relative to the path:

```python
def select_path(self, obstacles, robot_pose):
    for obs in obstacles:
        if dist < self.radius:
            side = self._side_of_path(obs, robot_pose)
            chosen = "right" if side == "left" else "left"
            return self.paths[chosen]
    return self.paths["center"]
```

The `_side_of_path` function determines whether the obstacle is to the left or right of the robot's heading. If the obstacle is on the left, we deviate right, and vice versa.

Path switching takes under 1 ms (a dictionary lookup and reference assignment). The 200 ms lag is completely eliminated. The robot can react to obstacles within a single control cycle (20 ms at 50 Hz).

I also added the "brake" path as a safety fallback. If an obstacle is detected very close (<0.2 m), the robot switches to the brake path, which is the current path with a velocity profile that decelerates to 0 at the obstacle location. This is a last-resort emergency stop.

### Remaining Issues

- The offset distance (0.3 m) is fixed. It should depend on the obstacle size, which we don't reliably estimate yet. For pillars of unknown diameter, 0.3 m might be too much clearance (wasting track space) or too little (if the pillar is larger than expected). The surprise rules could include a "wide pillar" variant that would require 0.5 m clearance.

- The precomputed paths are static. If the robot deviates significantly from the nominal path (e.g., after a slide or a previous avoidance maneuver), the precomputed offsets may not align with the actual robot position. The offset path assumes the robot is near the nominal path. If the robot is already 0.2 m off-center due to a previous avoidance, adding another 0.3 m offset could put the robot 0.5 m off-center, potentially hitting the track boundary.

- Switching paths causes a discontinuity in the steering command at the moment of switch. The Stanley controller processes the new path and adjusts steering smoothly (within one tick), but the discontinuous change in nearest waypoint can cause a steering jerk. I measured the steering rate at the switch: about 15°/s, which is within the servo's capability but produces a visible twitch.

- No logic to rejoin the nominal path after passing the obstacle. The robot follows the offset path indefinitely until a new obstacle is detected or the path resets during the next planning cycle. I should add a "rejoin" phase: after passing the obstacle, the robot should smoothly return to the nominal centerline over 1-2 meters.

### Files

- `obstacle_avoid.py` — DynamicObstacleAvoidance with precomputed alternative paths
- `avoidance_test.py` — Simulation test that places obstacles and verifies path switching timing
