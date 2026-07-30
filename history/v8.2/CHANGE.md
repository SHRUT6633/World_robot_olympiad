# v8.2 — Crab-Walk Steering Implementation

## What Changed

Today I implemented crab-walk steering — all four wheels turn to the same angle (say 30 degrees right) and the robot moves diagonally like a crab. This is essential for the WRO pillar avoidance challenge where we need to sidestep obstacles without changing the robot's heading.

The module is `steer_crab.py`. The geometry is trivial: all four wheels at angle θ, robot moves in the direction θ relative to its forward axis. The speed vector decomposes into forward and lateral components: `v_forward = v * cos(θ)`, `v_lateral = v * sin(θ)`. The robot's heading stays constant while it moves sideways.

## Errors Encountered

The crab-walk test revealed a subtle bug in the IMU fusion filter. When the robot moves sideways, the accelerometer senses lateral acceleration but the gyro senses no rotation. The Mahony filter in our IMU driver interprets this as a yaw rotation:

```
[IMU_FUSION] WARN: Yaw estimate drifting — correction applied: +0.7 deg
[IMU_FUSION] WARN: Yaw estimate drifting — correction applied: +1.2 deg
[IMU_FUSION] WARN: Yaw estimate drifting — correction applied: +1.8 deg
[CONTROL] ERROR: Yaw setpoint mismatch — current: 92.4 deg, target: 90.0 deg
[CONTROL] WARN: Applying yaw correction: steering angle delta = 1.8 deg
```

The robot was fighting itself. It would crab-walk sideways, but the IMU fusion thought it was rotating, so the controller would apply a counter-steering correction, which made the crab-walk diagonal instead of pure lateral. After 3 seconds of crab-walk, the yaw error accumulated to 12 degrees and the robot was crabbing at a 15-degree offset from the desired direction.

The root cause is that the Mahony filter uses accelerometer readings to correct gyro drift. During lateral acceleration (crab-walk), the accelerometer vector points sideways instead of down, and the filter interprets this as the frame having rotated. It's a known limitation of attitude filters under non-gravitational acceleration.

## The Fix

I added a yaw correction disable flag that gets set during crab-walk mode. The IMU fusion still estimates pitch and roll (which should remain zero during crab-walk), but yaw corrections are suspended. Yaw is propagated by gyro integration only.

```python
if steering_mode == "crab_walk":
    imu_filter.disable_yaw_correction = True
else:
    imu_filter.disable_yaw_correction = False
```

This is a bit dangerous because gyro bias will cause yaw drift over time. But for typical crab-walk durations (< 5 seconds), the drift is less than 1 degree, which is acceptable.

## Alternatives Considered

1. **Accelerometer gating**: Instead of fully disabling yaw correction, I could gate the accelerometer's influence based on the magnitude of the acceleration vector. If |accel| > 1.2g or the direction deviates from vertical by more than 30 degrees, reduce the yaw correction gain. This is more elegant but requires modifying the Mahony filter parameters at runtime, which is tricky.

2. **Switch to complementary filter during crab-walk**: The complementary filter is simpler and less susceptible to lateral acceleration because it weights the gyro more heavily. I could switch between Mahony and complementary filter based on steering mode. This would require maintaining two filter states, which doubles the memory footprint.

3. **Use magnetometer for yaw reference**: Our IMU has a magnetometer that isn't affected by lateral acceleration. However, the track has magnetic fields from the floor wiring that cause 5-10 degree errors in the magnetometer reading, so this isn't reliable.

## Testing

After the fix, crab-walk performs cleanly:
- Lateral movement accuracy: 0.02m error over 1m crab-walk
- Yaw drift: 0.3 degrees over 5 seconds
- No controller corrections during crab-walk
- Successful sidestep around pillar at 0.4m/s

## Lessons Learned

IMU fusion filters assume the only acceleration is gravity. Any significant lateral acceleration breaks this assumption. I need to be more careful about when yaw correction is active. The steering mode provides a clear signal for when lateral acceleration is expected — I should use this signal more broadly in the control system.
