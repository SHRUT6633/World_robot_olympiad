# v4.3 — Corner Detection

## What I Tried

The WRO 2026 track has 90° corners. The robot needs to detect when it's entering a corner, navigate through it, and detect when it's back on a straight. For this I used the MPU-6050 IMU's gyroscope to measure yaw rate and integrate it to get absolute yaw.

The approach:

1. Read gyro Z-axis at 100 Hz.
2. Integrate: `yaw += gyro_z * dt`.
3. Apply a simple complementary filter with the accelerometer for roll/pitch (not relevant here).
4. When integrated yaw exceeds 45° in either direction from the corner entry point, classify as "in corner."
5. When yaw returns to within 5° of the entry heading, classify as "corner complete."

## The Error — Gyro Drift

The first test run was a disaster. The robot approached a right-angle corner, turned, and then reported:

```
[CORNER] [15.234] Corner entry: yaw = 0.0°
[CORNER] [16.891] Yaw report: 85.3° — expected 90°
[WARN]  [16.892] Corner angle 85.3° outside tolerance, not counting as turn
```

And on the next corner:

```
[CORNER] [22.103] Corner entry: yaw = 12.7° (not reset!)
[CORNER] [23.554] Yaw report: 107.2° — expected 90°
```

The gyro drift was accumulating. Each successive corner had a different baseline because the bias error integrated over time. Between the first and second corner, the robot drove straight for about 5 seconds, but even that small interval introduced a 12.7° drift.

The MPU-6050 has a typical zero-rate offset of ±20 °/s, which is enormous. Even after calibration (taking the average of 100 readings at startup), the residual drift is about 0.5-1 °/s. Over a 10-second straight, that's 5-10° of drift.

## What I Changed

I reset the yaw to zero after every detected corner. Instead of maintaining an absolute global heading, I maintain a **relative yaw since the last corner**.

```python
if corner_detected:
    self.corner_entry_yaw = self.integrated_yaw
    self.waiting_for_exit = True

if self.waiting_for_exit:
    delta_yaw = abs(self.integrated_yaw - self.corner_entry_yaw)
    if delta_yaw > 85:
        self.completed_count += 1
        self.corner_entry_yaw = self.integrated_yaw
        # drift reset happens implicitly because we use delta from entry
```

This way the drift only accumulates during the corner itself (~1 second), not across the whole run. A 1-second drift at 1 °/s residual = ±1° error per corner, which is well within the ±5° tolerance.

## Alternatives Considered

- **Magnetometer (HMC5883L)**: Absolute heading from Earth's magnetic field. No drift! But the arena has metal pillars and wiring under the floor that distorts the field. In tests, the compass error was ±15° in some spots.
- **Visual odometry**: Could estimate rotation from optical flow. We'll try this in v4.9, but it's too heavy to be the primary corner detector now.
- **Mechanical bump switches on corners**: Not feasible — corners don't have physical triggers.
- **Kalman filter with accelerometer**: I could use the accelerometer to detect the centripetal acceleration during a turn and use that as an observation to correct the gyro bias. This is on the roadmap for a future version.

## Still Broken

- **Slow turns**: If the robot turns very slowly (< 30°/s), the gyro signal is close to the noise floor and the integrated angle is unreliable. The robot sometimes misclassifies a slow turn as a straight + small wiggle.
- **Bump on corners**: If the robot hits the wall at the corner entry, the accelerometer spike briefly corrupts the gyro reading via mechanical coupling. I've added a 50 ms debounce after any detected collision.

## Lesson Learned

Don't trust integrated gyro for absolute heading over more than a few seconds. The MPU-6050 bias stability is poor, and there's no substitute for periodic resets. For corner detection specifically, relative yaw per corner is good enough if you reset each time.
