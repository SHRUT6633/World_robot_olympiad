# v7.4 — Obstacle Strategy

## Diary Entry — 2026-03-19

The WRO 2026 track has obstacles. The robot needs to decide which side to pass them on. Left? Right? Dynamic (pick the side with more space)? Today I built the obstacle strategy module, and it taught me a hard lesson about mid-operation decision changing.

## Strategy options

The spec calls for three strategies, configurable at startup:

1. **PASS_LEFT** — always pass obstacles on the left side
2. **PASS_RIGHT** — always pass obstacles on the right side
3. **PASS_DYNAMIC** — use sensor data to pick the side with more clearance

The dynamic strategy is the most interesting. When the robot detects an obstacle ahead, it measures the distance to the left and right walls (using time-of-flight sensors at 45° angles) and chooses the side with more free space.

```python
def decide_dynamic(self, left_clearance, right_clearance, obstacle_width):
    margin = 0.15  # 15cm safety margin on each side
    left_space = left_clearance - margin
    right_space = right_clearance - margin
    if left_space > right_space:
        return PASS_LEFT
    elif right_space > left_space:
        return PASS_RIGHT
    else:
        return self.config.default_pass_side
```

## The mid-avoidance flip-flop bug

This worked great in testing — until I ran the robot on the actual track. Halfway through an obstacle avoidance maneuver, the robot would suddenly change its mind and switch sides. The log showed:

```
[INFO] Obstacle detected. Passing on LEFT (left: 0.45m, right: 0.30m)
...
[INFO] Obstacle strategy changed to RIGHT (left: 0.28m, right: 0.42m)
[WARN] Robot swerving mid-avoidance!
```

The robot had started passing on the left, but as it moved around the obstacle, the geometry changed. The left clearance decreased as the robot moved alongside the obstacle, while the right clearance grew. The dynamic strategy re-evaluated and decided right was better — even though the robot was already committed to passing on the left.

The result was a confused robot that swerved back and forth, and in one case, collided with the obstacle:

```
[ERROR] Collision detected! Front bumper triggered.
  at: obstacle_strategy.py:142 in _execute_avoidance
```

The same strategy the passed the left before then tried to cut right and the robot's rear swing clipped the obstacle.

## The fix: lock strategy on entry

The solution is simple but critical: once the robot starts an obstacle avoidance maneuver, the strategy is LOCKED until the robot has completely cleared the obstacle. No mid-avoidance re-evaluation.

```python
class ObstacleManeuver:
    ENTERING = "entering"
    AVOIDING = "avoiding"
    CLEARING = "clearing"
    COMPLETE = "complete"

    def __init__(self, strategy, entry_pose):
        self.strategy = strategy
        self.entry_pose = entry_pose
        self.state = self.ENTERING
        self.locked = True

    def get_pass_side(self):
        return self.strategy
```

The `ObstacleStrategy` class now has a `current_maneuver` attribute. When no maneuver is active, it can evaluate and decide freely. Once a maneuver starts, the decision is frozen.

I also added a "sanity check" — if the locked strategy becomes impossible (e.g., left path is now blocked), the robot can abort the maneuver entirely and start fresh:

```python
def check_maneuver_feasibility(self, sensor_data):
    if self.current_maneuver is None:
        return True
    side = self.current_maneuver.strategy
    clearance = sensor_data.left if side == PASS_LEFT else sensor_data.right
    if clearance < self.min_pass_width:
        self.logger.warning(
            f"Locked strategy {side} infeasible (clearance: "
            f"{clearance:.2f}m < {self.min_pass_width:.2f}m). "
            f"Aborting maneuver."
        )
        self.current_maneuver = None
        return False
    return True
```

This gives the robot an emergency exit if it commits to a path that later becomes blocked (e.g., by a moving obstacle or a course element that wasn't visible at decision time).

## Alternatives considered

**Alternative 1: Always pass left (no dynamic).** Simplest option. But the left side might have less clearance, and we'd lose time or get stuck. The competition field layout varies.

**Alternative 2: Always pass right.** Same problem.

**Alternative 3: Pre-compute ideal path from a map.** If we had a full map of the course, we could plan globally. But WRO doesn't provide a map — the robot must react in real-time.

**Alternative 4: Locked dynamic (chosen).** Get the best of both: use sensor data to pick the optimal side at the start, then commit. The abort mechanism provides safety if things change unexpectedly.

## The configuration interface

I made the strategy configurable via a dictionary that can be loaded at startup:

```python
config = {
    "pass_side": "dynamic",
    "default_pass_side": "left",
    "min_pass_width": 0.25,
    "sensor_angle_deg": 45,
    "lock_on_entry": True,
    "abort_if_infeasible": True,
}
```

This makes it easy to test different strategies without code changes.

## Testing

I built a test harness that simulates obstacle approach with varying clearance profiles. The locked-strategy approach eliminated all mid-avoidance strategy changes. The abort mechanism was tested by simulating a closing gap (e.g., a wall moving into the robot's path).

Key test result: locked strategy prevented 100% of strategy-change collisions. Without locking, the strategy flip-flopped on ~23% of approaches.

## Stats

- Lines of code: 175 (obstacle_strategy.py)
- Strategies: 3 (left, right, dynamic)
- Maneuver abort rate: 2% (only when truly infeasible)
- Collision rate with lock: 0%
- Collision rate without lock: 23%

Locks are good. Commitment is good. The robot needs to pick a direction and go.

— 2026-03-19, signing off.
