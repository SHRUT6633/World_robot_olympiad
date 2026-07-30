# v7.8 — Race Strategy

## Diary Entry — 2026-03-28

The robot can navigate the track, avoid obstacles, handle corners, detect direction, reverse when stuck, and park. But it's slow. Painfully slow. It drives at a conservative 0.15 m/s because that's the speed at which it can always handle any situation.

Today I built `race_strategy.py`, which adjusts the robot's target speed based on its confidence. When the robot is confident (track is clear, it knows where it is), it drives fast. When confidence drops, it slows down to handle the situation safely.

## The confidence model

Confidence is a value from 0.0 to 1.0. It starts at 0.5 (moderate confidence) and is adjusted up or down based on events:

```python
CONFIDENCE_EVENTS = {
    "lap_completed": +0.10,
    "corner_successful": +0.05,
    "obstacle_avoided": +0.02,
    "stuck_detected": -0.15,
    "obstacle_detected": -0.05,
    "alignment_failed": -0.10,
    "emergency_stop": -0.30,
}
```

The target speed is mapped from confidence:

```python
def get_target_speed(self):
    speed_range = self.max_speed - self.min_speed
    return self.min_speed + self.confidence * speed_range
```

At confidence 0.5 (initial): speed = 0.15 + 0.5 × (0.35 − 0.15) = 0.25 m/s
At confidence 1.0 (max): speed = 0.35 m/s
At confidence 0.0 (min): speed = 0.15 m/s

## The "too conservative" problem

When I first tested the race strategy, the robot never reached high speed. Here's a typical confidence trace from a 3-lap run:

```
Lap 1 start:  confidence 0.50, speed 0.25 m/s
Corner 1:     confidence 0.55, speed 0.26 m/s
Corner 2:     confidence 0.60, speed 0.27 m/s
Obstacle:     confidence 0.55, speed 0.26 m/s (avoided successfully!)
Corner 3:     confidence 0.60, speed 0.27 m/s
Lap 1 done:   confidence 0.70, speed 0.29 m/s
...
Lap 3 done:   confidence 0.85, speed 0.32 m/s
```

The robot completed 3 laps successfully, with zero errors, and only reached 0.32 m/s at the end. It was too conservative. The confidence gains from successes were small (+0.02 to +0.10), while any obstacle detection caused a -0.05 drop even when the obstacle was handled perfectly.

The problem: **every obstacle detection reduced confidence**, even though the robot always avoided them successfully. The "obstacle_detected" event is a penalty just for seeing an obstacle, regardless of outcome. This means the robot is punished for things it handles well.

## The fix: lap-completed confidence boost

The fix is to increase the confidence baseline after completing each successful lap. If the robot completes a full lap without getting stuck or needing emergency stop, it has proven its capability. The baseline shifts up:

```python
def _on_lap_completed(self):
    self.laps_completed += 1
    lap_bonus = 0.10 + 0.05 * self.laps_completed
    self.confidence = min(1.0, self.confidence + lap_bonus)
```

After lap 1: +0.15 boost
After lap 2: +0.20 boost
After lap 3: +0.25 boost

This means the robot accelerates over time. The first lap is cautious (learning the track), the second lap is faster (familiar), and the third lap is fastest (confident).

I also changed the obstacle penalty: instead of penalizing on detection, I only penalize if the obstacle causes a stuck event. If the robot avoids the obstacle smoothly, no penalty:

```python
CONFIDENCE_EVENTS = {
    "lap_completed": None,  # handled separately with progressive bonus
    "corner_successful": +0.05,
    "obstacle_avoided": +0.03,
    "stuck_detected": -0.15,
    "emergency_stop": -0.30,
    "alignment_difficulty": -0.05,
}
```

## The speed transition smoothing

Raw confidence-to-speed mapping caused the robot to jerk when confidence changed suddenly. An obstacle detected at speed 0.30 m/s would instantly drop to 0.22 m/s, causing a lurch. I added a low-pass filter on the speed output:

```python
def get_smoothed_speed(self, dt):
    target = self.get_target_speed()
    smoothing = 1.0 - math.exp(-dt / self.smoothing_time)
    self._current_speed += (target - self._current_speed) * smoothing
    return self._current_speed
```

With `smoothing_time = 0.5s`, speed changes are gradual, giving the robot's momentum time to adjust.

## Alternatives considered

**Alternative 1: Fixed speed.** Always drive at a constant conservative speed. Reliable but slow. Loses time in competition.

**Alternative 2: PID-based speed control.** Use a PID loop to adjust speed based on error from centerline. Requires reliable lane tracking, which we don't have yet.

**Alternative 3: Reinforcement learning.** Train a policy that maps sensor inputs to speed. Would be ideal but requires simulation and training time we don't have.

**Alternative 4: Confidence-based progressive speed (chosen).** Simple, interpretable, and effective. The lap-based bonus ensures the robot gets faster as it proves its capability.

## Testing

I compared 3-lap run times before and after the fix:

| Metric | Before fix | After fix | Improvement |
|--------|-----------|-----------|-------------|
| Avg speed lap 1 | 0.25 m/s | 0.25 m/s | — |
| Avg speed lap 2 | 0.27 m/s | 0.31 m/s | +15% |
| Avg speed lap 3 | 0.29 m/s | 0.34 m/s | +17% |
| Total run time | 38.2s | 32.1s | -16% |
| Stuck events | 0 | 0 | — |

The robot now completes runs 16% faster without any increase in stuck events. The lap-based confidence boost was the key insight.

## Stats

- Lines of code: 116 (race_strategy.py)
- Confidence range: 0.0–1.0
- Speed range: 0.15–0.35 m/s
- Lap 1 speed: 0.25 m/s (cautious)
- Lap 3 speed: 0.34 m/s (confident)
- Time improvement: 16%

The robot learns to trust itself. One more component to go.

— 2026-03-28, signing off.
