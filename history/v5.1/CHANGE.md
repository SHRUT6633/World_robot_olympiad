# v5.1 — Heading from Magnetometer

**Theme:** "Which way is north?"

Dead reckoning's fatal flaw is heading drift. Even with perfect encoders, a 1° heading error produces 1.7cm of lateral error per meter of forward travel. Over a WRO course with 5m of driving, that's 8.5cm of error from heading alone. Fix the heading, fix the biggest error source.

Enter the magnetometer. The BNO055 on our board has a built-in magnetometer that can sense the Earth's magnetic field and give us absolute heading. No drift. No integration. Just "north is that way."

I wrote `mag_heading.py` to fuse gyro yaw (short-term stability) with magnetometer heading (long-term absolute reference). The fusion is a simple complementary filter: `heading = alpha * gyro_heading + (1-alpha) * mag_heading`. Gyro gives us smooth, low-latency updates at 100Hz. Magnetometer gives us the anchor at 20Hz.

The first test was promising. I spun the robot slowly by hand while logging. Gyro-only track diverged by 15° after 30 seconds. Mag + gyro fusion stayed within 2° of true heading. Victory.

Then I mounted the sensor back on the robot chassis and ran the motors.

Everything broke.

The magnetometer reading jumped by 40° the instant the motors started. The magnetic field from the DC motors overwhelmed the Earth's field. Worse, the interference was proportional to throttle — at 30% PWM the offset was 5°, at 70% it was 25°, at 100% it was 40°. And it wasn't a simple constant offset because the magnetic field changes with motor load, battery voltage, and direction of travel.

```
[MAG] Static heading: 87.3° (earth field ~45µT)
[MAG] Motors ON (50%): heading 112.8° (offset +25.5°)
[MAG] Motors ON (80%): heading 132.1° (offset +44.8°)
[MAG] Motors OFF: heading 86.9° (back to normal)
```

I tried software filtering first. A 2Hz low-pass on the magnetometer. It helped smooth the noise but the DC offset remained. No amount of filtering removes a DC bias.

I tried mounting the magnetometer on a mast 10cm above the motors. It reduced the interference from 40° to 12°. Better, but 12° is still 10cm lateral error per meter.

I tried hard-iron calibration (rotating the robot in figure-8s while recording max/min readings). This corrected for static offsets from the robot's own metal structure, but the motor interference is dynamic — it changes with current draw.

The fix was pragmatic: disable magnetometer correction while the motor is running. Use the gyro to propagate heading during motion, then correct with the magnetometer when the robot is stationary. This is what many drones do for their compasses — they simply don't trust the compass during high-throttle maneuvers.

I added a `motor_running` flag. When True, the filter relies entirely on gyro integration. When False, it slowly corrects toward the magnetometer reading. The correction rate is governed by a `mag_correction_gain` that ramps up over 500ms after motors stop (to avoid snapping the heading when gyro has drifted).

Implementation detail: when the motor stops, I feed the magnetometer heading into a washout filter: `heading_correction = (mag_heading - gyro_heading) * gain`. The gain starts at 0.0 and ramps to 0.1 over 500ms. This gives a smooth convergence.

The downside: gyro still drifts during motor runtime. The IMU's gyro has a bias stability of about 1°/s, meaning over a 30-second WRO run we could see 30° of gyro-only drift. That's unacceptable for the full run. But it's good enough for segments between corrections. The plan is to stop briefly at waypoints, let the magnetometer re-anchor, then continue.

Not ideal. But workable. Next version (v5.2) will add accelerometer data for a full attitude solution, which will help distinguish heading changes from gyro bias more effectively.

Key files:
- `mag_heading.py` — Complementary filter fusing gyro and magnetometer
- `motor_interference_test.py` — Test script that characterized the interference pattern
