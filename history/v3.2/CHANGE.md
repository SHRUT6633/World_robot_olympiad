# v3.2 — Complementary Filter for Pitch and Roll

## What Changed
With calibrated gyro bias and accelerometer scale factors from v3.1, we now have two ways to measure orientation: the accelerometer gives us instantaneous pitch and roll via the gravity vector (atan2(ax, az) for pitch, atan2(ay, az) for roll), and the gyro gives us rate of rotation that we can integrate over time. Both have complementary weaknesses: accelerometer is noisy in the short term (vibrations from motors) but stable in the long term (always points to gravity), while gyro is smooth in the short term but drifts in the long term due to bias integration error.

The complementary filter fuses them: `angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle`. This gives us a drift-free, low-noise pitch and roll estimate at ~95 Hz.

`complementary.py` reads the filter parameters (`alpha` and `dt`) from a config dict, loads the IMU calibration JSON from v3.1, and implements a `ComplementaryFilter` class with `update(ax, ay, az, gx, gy, gz, dt)` that returns pitch and roll in radians.

## Why
Without the complementary filter, we couldn't use the IMU for anything useful during robot motion. The accelerometer alone is useless while driving because motor vibrations produce 200-500 mg of noise (we measured it: with motors at 50% PWM, accel noise increased 5x). The gyro alone would drift by tens of degrees in a minute. The complementary filter is the standard solution for exactly this problem, and it's simple enough to implement in a single class with no external dependencies beyond NumPy.

## Errors Encountered

### Filter Gain Alpha=0.98 Causes 1 Second Lag
Our first implementation used alpha=0.98, which is the value recommended in most online tutorials (including the venerable "A Guide To using IMU in Embedded Applications" by Starlino). We tested it by tilting the robot rapidly by 30 degrees and watching the filter output. The filter took 1.0-1.2 seconds to converge to the correct angle. For a robot navigating a WRO course at 0.5 m/s, that's 0.5 meters of travel before the filter catches up. Way too slow for obstacle avoidance.

```
PERF: Pitch step response (30 deg): 1.15s to 95% of final value
PERF: Roll step response (30 deg): 0.98s to 95% of final value
ERROR: Lag causes robot to overshoot turns by ~40 cm
```

I traced the issue. With alpha=0.98, the filter gives 98% weight to the gyro integration and only 2% to the accelerometer correction. At 100 Hz, the time constant is `tc = (alpha * dt) / (1 - alpha) = (0.98 * 0.01) / 0.02 = 0.49s`. But in practice, because the gyro integration accumulates error and the accelerometer correction is so weak, the effective time constant is closer to 1 second.

**Fix:** We decreased alpha to 0.92. This gives a time constant of `(0.92 * 0.01) / 0.08 = 0.115s`. Measured step response: 0.21 seconds to 95%.

The trade-off: slight drift when the robot is stationary (about 0.5 deg/s instead of 0.1). This is acceptable for a fast-moving robot. We can also dynamically adjust alpha based on detected motion: when the accelerometer variance is high (motors on), favor the gyro (alpha=0.95); when variance is low (stationary), favor the accelerometer (alpha=0.85).

```python
# Before (too slow)
alpha = 0.98
# After (acceptable)
alpha = 0.92
```

### Gimbal Lock at Pitch ±90 Degrees
When we pitched the robot up 90 degrees (nose vertical), the filter output became chaotic. Roll jumped wildly. This is standard gimbal lock from using Euler angles—when pitch is 90°, roll and yaw axes align and you lose a degree of freedom. The atan2-based accelerometer angle calculation also becomes singular.

**Fix:** We switched to storing orientation as a quaternion internally. The complementary filter runs on quaternions: compute the error quaternion between the accelerometer-estimated attitude and the current gyro-integrated attitude, then slerp by (1-alpha). This avoids gimbal lock entirely. We output pitch and roll only at the API boundary, where we apply a safe conversion with singularity checks.

### NaN Propagation
If the accelerometer ever reports zero on all axes (which can happen if the I2C read fails and we get zeros), the `atan2(ay, az)` becomes `atan2(0, 0)` = NaN. Once a NaN enters the filter, it propagates forever.

```
ERROR: Filter output at t=12.5s: pitch=NaN, roll=NaN
```

**Fix:** Added a validity check: if `sqrt(ax^2 + ay^2 + az^2)` is outside [0.5, 1.5]g, we skip the accelerometer update and only use gyro integration for that sample. This is actually better than the complementary filter in the normal case because a free-fall or hard impact corrupts the gravity vector anyway.

```python
accel_norm = np.sqrt(ax**2 + ay**2 + az**2)
if 0.5 < accel_norm < 1.5:
    # Accelerometer is reliable, use it
    accel_pitch, accel_roll = accel_to_angle(ax, ay, az)
else:
    # Skip accelerometer update this frame
    accel_pitch, accel_roll = None, None
```

## Alternatives Considered
- **Madgwick filter**: More sophisticated, uses gradient descent to compute orientation from gyro, accel, and optionally magnetometer. It's been shown to be more accurate than complementary filter at the same computational cost. We chose to start with complementary because it's easier to understand and debug. We may adopt Madgwick in v3.4 if we add magnetometer heading.
- **Kalman filter**: A full 6-state linearized Kalman filter for attitude estimation. Too much complexity for now. The complementary filter is a special case of a Kalman filter with fixed gains, and for our use case it works well enough.
- **Mahony filter**: Similar to Madgwick but uses proportional-integral feedback on gyro bias. We'll revisit if we see bias drift issues.

## Current Status
`complementary.py` outputs stable pitch/roll at 95 Hz with 200 ms step response, no gimbal lock, and NaN-safe. The filter converges to within 0.5 degrees of the true angle after 0.2 seconds. Next step: add magnetometer heading (v3.3) to get yaw, completing the full 3D orientation.
