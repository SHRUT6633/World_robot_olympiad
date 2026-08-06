# v5.2 — Complementary Filter Full 6-DoF Attitude

**Theme:** "Don't just know north — know which way is up."

Heading (yaw) alone isn't enough. The robot drives on flat ground, but the IMU reports in its own body frame. If the robot tilts (acceleration, uneven floor, bump), the magnetometer heading becomes inaccurate because the sensor plane is no longer horizontal. We need full roll, pitch, and yaw.

I implemented a full complementary filter combining all three IMU sensors: accelerometer (gravity vector for roll/pitch reference), gyroscope (angular velocity for high-rate integration), and magnetometer (magnetic field for yaw reference). The filter structure is standard: gyro integrates attitude at 100Hz, accelerometer and magnetometer provide corrections at a lower rate.

The accelerometer gives us roll and pitch by measuring which way gravity points. `roll = atan2(ay, az)`, `pitch = atan2(-ax, sqrt(ay^2 + az^2))`. Simple, except when the robot is accelerating — then the accelerometer measures `acceleration + gravity`, not just gravity. The complementary filter handles this by heavily low-passing the accelerometer contribution (trust it for long-term stability, ignore it for short-term dynamics).

The magnetometer gives yaw, but only after tilt-compensation. The raw magnetometer reading must be rotated back to the horizontal plane using the current roll/pitch estimate before computing heading. This is critical: if the robot is pitched 10° forward (braking), the raw magnetometer yaw is off by up to 15°.

The first test was smooth. I held the robot in my hand and slowly rotated it through all axes. The filter tracked beautifully — smooth gyro response with no drift, corrected by accel and mag at low frequencies.

Then I spun it fast.

At ~90°/s rotation rate, the filter started oscillating. By 120°/s, it diverged entirely — the roll estimate flipped 180° and stayed there.

```
[FILTER] Rotation rate: 85°/s — tracking OK, 2° error
[FILTER] Rotation rate: 95°/s — oscillation starting, 5° error
[FILTER] Rotation rate: 120°/s — DIVERGED. Roll: 187°, Pitch: -23°
```

The root cause: during fast rotation, the gyroscope integration accumulates error rapidly, and the complementary filter's correction gain is too slow to catch it. The accelerometer correction assumes the measured acceleration is mostly gravity, but during fast rotation, centripetal acceleration (`a = ω²r`) corrupts the gravity estimate. At 120°/s with the IMU 3cm from the rotation center, centripetal acceleration is about `(2.09 rad/s)² * 0.03m = 0.13 m/s²`, or about 1.3% of gravity — not huge. The bigger issue is gyro saturation and the filter's fixed gain structure.

I tried reducing the gyro trust gain. That made slow tracking worse. I tried increasing the accelerometer cutoff frequency. That introduced vibration noise from the robot structure.

The fix: dynamically reduce gyro trust when angular velocity exceeds a threshold. I added a `gyro_trust` multiplier that scales with the inverse of angular velocity:

```
if angular_rate > RATE_THRESHOLD:
    gyro_trust = RATE_THRESHOLD / angular_rate
```

Above 90°/s, gyro trust drops linearly. At 180°/s, gyro trust is 0.5 — the filter relies 50% on gyro, 50% on accelerometer/magnetometer prediction. This prevents gyro integration from running away during fast spins while still using it for short-term dynamics.

RATE_THRESHOLD = 90°/s (1.57 rad/s) is the empirically determined point where gyro error starts to dominate.

I also added gyro bias estimation. Over 10 seconds of quiescent data, I compute the mean gyro reading and subtract it from future measurements. This reduced the slow drift from 1°/s to about 0.1°/s.

The filter now passes the fast-spin test. Up to 150°/s, tracking stays within 5° of true attitude. The WRO robot shouldn't exceed 60°/s in normal operation, so we have margin.

Key files:
- `complementary_full.py` — Full 6-DoF complementary filter
- `attitude_test.py` — Test harness for rotation rate characterization
