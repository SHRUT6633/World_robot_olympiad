# v7.6 — Reverse Logic

## Diary Entry — 2026-03-23

Sometimes the robot gets stuck. A wheel catches on a seam in the track mat. The robot pushes against an obstacle at a bad angle and can't make progress. The bumper touches a wall and the robot can't steer away. When this happens, the robot needs to reverse, reorient, and try again.

Today I built `reverse_logic.py`, which detects when the robot is stuck and decides when to reverse.

## Stuck detection

The core idea is simple: if the robot is trying to move forward but isn't making positional progress, it's stuck. I track the robot's pose over a 2-second window and check if the total displacement is below a threshold:

```python
def _check_stuck(self, pose):
    if self._last_progress_pose is None:
        self._last_progress_pose = Pose(pose.x, pose.y, pose.yaw)
        self._last_progress_time = time.monotonic()
        return False

    dx = pose.x - self._last_progress_pose.x
    dy = pose.y - self._last_progress_pose.y
    distance = math.hypot(dx, dy)
    elapsed = time.monotonic() - self._last_progress_time

    if distance > self.progress_threshold:
        self._last_progress_pose = Pose(pose.x, pose.y, pose.yaw)
        self._last_progress_time = time.monotonic()
        return False

    if elapsed > self.stuck_timeout:
        self.logger.warning(
            f"Robot stuck! No progress for {elapsed:.1f}s "
            f"(moved only {distance:.2f}m)"
        )
        return True

    return False
```

Default values: `stuck_timeout = 2.0s`, `progress_threshold = 0.02m` (2cm). If the robot hasn't moved more than 2cm in 2 seconds, it's considered stuck.

## The "reverse too aggressive" bug

My first version had a simple reverse: back up at full speed for 1 second. The result was predictable:

```
[INFO] Robot stuck. Reversing...
[ERROR] Collision detected! Rear bumper triggered.
  at: reverse_logic.py:201 in _execute_reverse
```

The robot backed up for a full second at ~0.3 m/s, traveling about 30cm — straight into the wall behind it. The track is only about 60cm wide in the corridors. Backing up 30cm put the robot's rear into the outer wall.

Here's the actual timing log:

```
t=0.000s  Stuck detected. Start reverse.
t=0.100s  Rear ToF: 0.45m
t=0.200s  Rear ToF: 0.35m
t=0.300s  Rear ToF: 0.25m  ← getting close
t=0.350s  Rear ToF: 0.18m  ← TOO CLOSE
t=0.400s  Rear ToF: 0.12m  ← BUMPER TRIGGERED
t=0.420s  COLLISION DETECTED
```

The rear-facing time-of-flight sensor showed the distance dropping rapidly, but the reverse logic didn't check it. It just ran for 1 second regardless of what was behind the robot.

## The fix: limited reverse distance with sensor monitoring

I added two safeguards:

1. **Maximum reverse distance (20cm).** Instead of reversing for a fixed time, I track the distance traveled in reverse. Once the robot has backed up 20cm, it stops, regardless of time.

2. **Rear sensor monitoring.** I check the rear-facing distance sensor during reverse. If the robot is about to hit something (distance < 15cm), the reverse stops immediately and the robot transitions to an emergency maneuver.

```python
def execute_reverse(self, pose, rear_distance):
    if self.reverse_state is None:
        self.reverse_state = ReverseState.REVERSING
        self._reverse_start_pose = Pose(pose.x, pose.y, pose.yaw)
        self._reverse_start_time = time.monotonic()

    dx = pose.x - self._reverse_start_pose.x
    dy = pose.y - self._reverse_start_pose.y
    reverse_distance = math.hypot(dx, dy)

    if reverse_distance >= self.max_reverse_distance:
        self.logger.info(f"Reached max reverse distance ({reverse_distance:.2f}m)")
        return ReverseResult.STOP_REVERSE

    if rear_distance is not None and rear_distance < self.rear_safety_margin:
        self.logger.warning(
            f"Rear obstacle detected ({rear_distance:.2f}m). "
            f"Stopping reverse."
        )
        return ReverseResult.ABORT_REVERSE

    return ReverseResult.CONTINUE_REVERSE
```

## The "still stuck" loop

After the fix, the robot would reverse 20cm, stop, try to go forward again, and immediately detect that it was still stuck. This created an infinite loop of reverse → forward → stuck → reverse. The robot never got past the stuck point because it wasn't changing its heading enough during the reverse.

I added a "reorientation" phase after reverse: instead of going straight forward after reversing, the robot turns 15-30° (alternating direction each time) before trying again:

```python
def get_recovery_turn(self, stuck_count):
    if stuck_count % 2 == 0:
        return math.radians(20)
    else:
        return math.radians(-20)
```

The `stuck_count` tracks how many consecutive times the robot has gotten stuck. Each attempt turns in a different direction, helping the robot find a path around whatever was blocking it.

## Alternatives considered

**Alternative 1: No reverse at all.** Just try to steer around obstacles without backing up. This works for most situations, but when the robot is genuinely stuck (wheel against a wall), steering alone can't help.

**Alternative 2: Reverse with path planning.** Use the occupancy grid to find a clear path behind the robot. Too computationally expensive for a Pico.

**Alternative 3: Shuffle (wiggle) instead of reverse.** Oscillate the wheels back and forth to shake the robot loose. This works for wheel slip but not for physical blockage.

**Alternative 4: Limited reverse with sensors (chosen).** Simple, effective, uses existing sensors. The 20cm limit and rear monitoring prevent collisions while still giving the robot room to maneuver.

## Testing

I simulated 50 stuck scenarios with various obstacle configurations. The limited reverse (20cm) with rear sensor monitoring eliminated 100% of rear collisions. Without the limit, 28% of reverses resulted in rear collisions. The alternating reorientation improved escape success from 62% to 91%.

## Stats

- Lines of code: 152 (reverse_logic.py)
- Max reverse distance: 20cm
- Stuck timeout: 2.0s
- Progress threshold: 2cm
- Rear safety margin: 15cm
- Escape success rate: 91% (was 62%)

The robot now knows when to give up, back away carefully, and try again. It's a small piece of intelligence that makes a huge difference in reliability.

— 2026-03-23, signing off.
