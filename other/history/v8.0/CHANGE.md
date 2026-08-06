# v8.0 — Same-Phase Steering Implementation

## What Changed

Today I implemented the same-phase steering mode for our WRO 2026 robot. This is the most fundamental steering mode — all four wheels turn in the same direction, causing the robot to pivot around an imaginary center point somewhere along its longitudinal axis. The target was to achieve a minimum turning radius of 0.8 meters.

I created a new module called `steer_same.py` that takes a desired turning radius and computes the steering angle for each wheel. The geometry is actually simpler than it sounds: for same-phase steering, all wheels need to point along concentric arcs around the same center of rotation. The inner wheels (left side when turning left) need to turn more sharply than the outer wheels, but since our design uses independent wheel steering, I can just set all four to the same angle and let the speed differential handle the rest. Wait — no, that's not right. Same-phase means all wheels point the same direction, and the robot turns because the steering angle creates a lateral force component. It's like how a car turns, but all four wheels steer.

I derived the steering angle from the turning radius using the formula: `steering_angle = atan(wheelbase / (2 * turning_radius))`. This gives the angle for the front wheels. For same-phase I set all four to this value.

## Errors Encountered

The first test run was a disaster. At full lock (30 degrees), the inner wheels were making this horrible scraping noise. I checked the logs and found lateral slip values spiking to 0.8 — way beyond the acceptable 0.15 threshold.

```
[WHEEL_SLIP] WARN: Left-front wheel slip ratio = 0.82 (threshold: 0.15)
[WHEEL_SLIP] WARN: Left-rear wheel slip ratio = 0.79 (threshold: 0.15)
[ODOMETRY] ERROR: Position uncertainty exceeded 0.5m — odometry unreliable
```

The problem is kinematic: when all four wheels steer the same direction, the inner wheels follow a tighter arc than the outer wheels. At 30 degrees, the inner wheels' required radius is about 0.55m but the outer wheels are at 0.8m. The Ackermann condition isn't satisfied because all wheels are parallel, so the inner wheels scrub sideways to compensate.

## The Fix

I limited the maximum steering angle to 25 degrees. This reduces the lateral force on the inner wheels and keeps the slip ratio below 0.12 even at minimum radius. The trade-off is that the minimum turning radius increased slightly from my calculated 0.75m to about 0.82m, which still meets the 0.8m requirement.

The fix was a single line change in the config:
```python
MAX_STEERING_ANGLE = math.radians(25)  # was 30
```

## Alternatives Considered

1. **Ackermann steering geometry**: I could have calculated individual wheel angles so each wheel follows its natural arc. This would eliminate scrub entirely, but it requires different angles for each wheel, which complicates the control logic. We'd need per-wheel steering servo calibration and the mechanical linkage would be more complex. I decided against it for v8.0 because we need this working for the track tests tomorrow, and Ackermann would require re-machining the steering linkages.

2. **Speed differential**: I considered reducing the inner wheel speed to compensate for the tighter arc. This works for skid-steer robots but our hub motors are designed for synchronized movement, and differential speed introduces torque ripple in the steering servos. I tested it briefly and the steering servos started oscillating at 3Hz, which would destabilize the platform.

3. **Reduced speed at full lock**: Similar to what I ended up doing but instead of limiting angle, I limited speed to 0.2m/s when angle exceeds 25 degrees. This technically works but it means the robot slows down in the middle of a turn, which could confuse the path planner. The angle limit approach is cleaner because the path planner always knows the turning radius regardless of speed.

## Testing

After the fix, I ran 50 consecutive full-lock turns:
- Max slip ratio: 0.12 (left front)
- Average turning radius: 0.82m
- Position uncertainty after 10m: 0.03m
- No wheel scrub noise audible

The robot now handles consistent turns without destroying its tires. Moving on to v8.1 where I'll implement opposite-phase steering for tighter maneuvers.

## Lessons Learned

Steering geometry is non-trivial. The simple "set all wheels to same angle" approach has kinematic limitations that manifest as scrub. I should probably build a proper Ackermann solution in a later version, but for now the angle limit is sufficient for the competition track which mostly uses gentle curves.
