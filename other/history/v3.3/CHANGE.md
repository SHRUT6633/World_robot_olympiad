# v3.3 — Magnetometer Heading

## What Changed
The complementary filter (v3.2) gave us pitch and roll, but yaw was still drifting. Gyro yaw integration accumulates error, and without an absolute yaw reference, the robot can't tell which way it's facing. Enter the QMC5883L magnetometer—a 3-axis magnetic field sensor that costs $3 and communicates over I2C. It measures the Earth's magnetic field, and by taking the arctangent of the X and Y horizontal components, we get the compass heading.

`heading.py` reads the QMC5883L's X, Y, Z registers (16-bit, ±8 gauss range), applies hard-iron distortion calibration (see below), and computes heading as `atan2(Y, X)` in degrees. The sensor is mounted on the front of the robot, aligned with the robot's forward axis.

We also integrated the magnetometer into a Madgwick filter (upgraded from complementary filter) that now fuses gyro, accelerometer, and magnetometer for full 3D orientation: pitch, roll, and yaw. The Madgwick filter uses gradient descent to compute the quaternion that best explains all three sensor measurements simultaneously.

## Why
Compass heading is the only way to get absolute yaw without external infrastructure (no GPS indoors, no APR tags yet). The WRO 2026 rulebook requires robots to follow a path that includes 90-degree turns at specific landmarks. Without heading, we can't execute a precise 90° turn—we'd have to rely on odometry, which slips. With heading, we can turn until the compass reads 90° from the start heading.

## Errors Encountered

### Hard Iron Distortion From Motor Magnets
The biggest problem: when we mounted the QMC5883L on the robot, the heading was off by 30-50 degrees when the motors were running. Even with motors off, the heading was wrong by 10-20 degrees depending on the robot's orientation.

```
ERROR: Heading with motors off: 45° (should be 90° based on visual alignment)
ERROR: Heading with motors on (50% PWM): 78° (varies with motor load)
ERROR: Heading after rotating 360°: min=20°, max=320° (range 300° instead of 360°)
```

This is hard-iron distortion: the permanent magnets in the DC motors add a constant magnetic field vector that shifts the sensor's measurement origin away from (0,0,0). The effect is visible as a shifted circle when plotting X vs Y over a 360° rotation.

**Fix:** Implement calibration. We rotate the robot 360° manually (or drive it in a circle on the spot), recording all X, Y samples. Then compute:
- Offset_X = (max_X + min_X) / 2
- Offset_Y = (max_Y + min_Y) / 2

These offsets are subtracted from all subsequent readings. This is the standard 2D hard-iron calibration (we ignore Z for heading).

```python
# Calibration
cal = {"x_offset": (x_max + x_min) / 2, "y_offset": (y_max + y_min) / 2}
# Correction
corrected_x = raw_x - cal["x_offset"]
corrected_y = raw_y - cal["y_offset"]
heading = atan2(corrected_y, corrected_x) * 180 / pi
```

After calibration, the heading error dropped to ±5 degrees with motors off, ±8 degrees with motors on. The remaining error is soft-iron distortion (varying with direction because the robot's steel chassis changes the field geometry), which requires a more complex ellipsoid fit. We'll address that in a future version if needed.

### QMC5883L Data Rate Too Slow
The QMC5883L defaults to 10 Hz data rate (register 0x09, MODE_CONTROL). At 10 Hz, we get a heading update every 100 ms, but our filter runs at 100 Hz. This means 90 out of 100 filter iterations have no new magnetometer data, and the heading estimate relies entirely on gyro integration.

**Fix:** Set the data rate register to 100 Hz (MODE_CONTROL = 0x1D for 100 Hz continuous). We also set the oversampling to 512 (register 0x09 bits 6-7 = 0b11 for OS512) for better noise performance at the cost of higher power consumption.

```python
# Set 100 Hz continuous mode, OS512
bus.write_byte_data(QMC_ADDR, 0x09, 0x1D)
```

### Heading Calculation Wrong Sign Convention
Our first heading output was backwards. When we pointed the robot north, it read 180°. When we rotated clockwise, the heading decreased. This is because the QMC5883L's Y axis points to the magnetic north pole by default, but our robot's forward direction is the sensor's X axis. We needed atan2(Y, X) but with sign adjusted for the right-hand rule (clockwise increase).

**Fix:** We re-read the QMC5883L datasheet and determined the axis orientation relative to our robot. The chip's Y points left, X points forward. So heading = atan2(-Y, X) where Y is negated because positive Y is magnetic east (left), but in navigation, heading increases clockwise (east is 90°, south is 180°, west is 270°).

```python
heading = (np.arctan2(-y_corrected, x_corrected) * 180.0 / np.pi) % 360.0
```

## Alternatives Considered
- **HMC5883L**: The older, more popular Honeywell magnetometer. It's more accurate (0.1° resolution vs 0.5° for QMC), but it's also more expensive ($15 vs $3) and harder to source. We had QMCs in stock.
- **IST8310**: Used in many Pixhawk flight controllers. Better temperature stability. Would require different I2C library. Not worth the effort.
- **Soft-iron calibration**: Full 3D ellipsoid fitting with the Z-axis data. We'll implement this if we need better than ±5° accuracy. For WRO, ±5° is probably acceptable for 90° turns.
- **GPS heading**: Using dual GPS antennas for yaw. Too expensive and too slow. GPS update rate is 10 Hz max.
- **Visual odometry heading**: Using optical flow sensor (PMW3901). Unreliable on featureless track surfaces.

## Current Status
`heading.py` provides compass heading at 100 Hz with ±5° accuracy. The Madgwick filter in `complementary.py` was updated to use magnetometer data for yaw correction. The robot can now execute 90° turns by monitoring the heading delta. `imu_calib.json` now includes `mag_calib` with `x_offset`, `y_offset`. Next step: add Time-of-Flight distance sensors (v3.4) for obstacle detection.
