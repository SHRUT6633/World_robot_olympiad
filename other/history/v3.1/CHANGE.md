# v3.1 — IMU Calibration

## What Changed
After collecting raw IMU data in v3.0, we had a CSV file full of numbers but no way to distinguish "the robot is stationary" from "the gyro thinks we're slowly spinning even though we're not." That's bias. Every MEMS gyro has a non-zero output at rest, and it changes with temperature, board stress, and time. The accelerometer has scale factor errors—if the sensitivity is off by 2%, our pitch estimate will be off by 2%.

`calibrate_imu.py` addresses both. It collects 1000 stationary readings (about 10 seconds at ~95 Hz), averages the gyro values to get bias, and computes accelerometer scale factors by measuring +1g on each axis when the robot is oriented with that axis pointing straight up. The script saves the calibration parameters to a JSON file (`imu_calib.json`) that every downstream module reads on startup.

The calibration procedure:
1. Place robot on a known-level surface (we used a machined aluminum plate and a bubble level).
2. Run calibration script. It computes gyro bias as the mean of N=1000 stationary gyro readings.
3. For accelerometer scale factors: we can't easily do a 6-position test without a precision fixture, so we approximate by assuming Z has correct scale and using the ratio of measured gravity on each axis to the expected 1g. This is a shortcut but works for our purposes.

## Why
Raw gyro readings drift by 0.5-2 deg/s even at rest. Without subtracting bias, integrating gyro data in the complementary filter (v3.2) would cause pitch/roll to drift 30-120 degrees per minute. That's useless for a robot that needs to navigate a WRO course precisely. The accelerometer also needs scaling correction—our initial log showed Z reading 1.03g when the robot was level, which would make the complementary filter think the robot is tilted by arccos(1/1.03) ≈ 14 degrees.

## Errors Encountered

### Bias Drifts With Temperature
The first calibration worked great. We got gyro bias of [-0.87, 1.12, -0.54] deg/s. But when we left the robot running for 20 minutes, the bias drifted to [-0.92, 1.31, -0.61]. Then we took the robot outside (30°C vs 25°C lab) and got [-1.02, 1.45, -0.72]. The MPU6050 datasheet specifies ±0.05 deg/s/°C typical temperature coefficient. At first we thought we had a defective unit.

```
WARNING: Gyro bias X changed from -0.87 to -0.92 after 20 min
WARNING: Gyro bias X changed from -0.87 to -1.02 after moving outside
ERROR: Pitch drift after 60s: 15 degrees (expected < 3)
```

We also noticed the internal temperature register (register 0x41, the MPU6050 has a built-in thermometer) changed from 28°C to 34°C. The bias was clearly temperature-dependent.

**Fix:** We store bias at the current temperature, and after calibration, we continuously monitor the temperature register. If the temperature changes by more than 5°C from the calibration temperature, we trigger a recalibration. Since recalibration requires the robot to be stationary, we added a "stationary detection" heuristic—if gyro variance over the last 100 samples is below 0.1 deg/s, the robot is assumed stationary and we accumulate new bias samples.

```python
if abs(temp - calib_temp) > 5.0:
    print("Temperature change >5C, recalibrating gyro bias")
    recalibrate_bias()
```

This isn't ideal because recalibrating during a run could corrupt state, but for now it happens rarely enough (the temperature stabilizes after 5 minutes) that it's acceptable.

### Accelerometer Scale Factor Convergence
Our first attempt at computing scale factors used only 100 samples, but the results were noisy: scale factor for X varied ±0.03 between runs. The accelerometer readings have white noise of about 5 mg RMS at ±2g range.

**Fix:** Bump the averaging window to 1000 samples per orientation. This reduces the standard error to 5 mg / sqrt(1000) ≈ 0.16 mg, giving scale factor precision of about 0.02%.

### Wrong Sign on Gyro Bias
At one point we subtracted the bias instead of adding it. Our gyro bias was positive (1.12 deg/s on Y), but after integration, the angle was negative. We stared at the code for an hour. The issue: `gyro_rate = raw_gyro - bias`. If raw gyro reads 1.12 deg/s when stationary, and bias is 1.12, then gyro_rate = 0. Correct. But we had `gyro_rate = bias - raw_gyro`. The raw gyro read 1.12, bias is 1.12 → 0. Still correct? No, because when the robot actually rotates, say raw_gyro = 2.0, then gyro_rate = 1.12 - 2.0 = -0.88 deg/s, when it should be +0.88. The sign was flipped.

```python
# Wrong
gyro_rate_x = gyro_bias["x"] - raw_gx
# Correct
gyro_rate_x = raw_gx - gyro_bias["x"]
```

## Alternatives Considered
- **Factory calibration**: The MPU6050 stores factory-calibrated values in registers 0x06-0x07 (self-test response). We could trust those, but they don't account for board stress from soldering.
- **Ellipsoid fitting**: For the accelerometer, a full 3D ellipsoid fit requires 6+ known orientations. We don't have a precise fixture, so we stuck with the simpler 1-axis-at-a-time approach.
- **External temperature sensor**: We could wire a DS18B20 to the IMU board with thermal epoxy, but the MPU6050's internal temperature sensor, while low resolution (1°C per LSB), is good enough for the 5°C threshold.
- **Continuous online calibration**: The ZUPT (Zero Velocity Update) technique used in foot-mounted IMUs. This would detect stationary periods automatically and update bias on the fly. Too complex for now.

## Current Status
`calibrate_imu.py` outputs `imu_calib.json` with `gyro_bias`, `accel_scale`, `calib_temp`, and `timestamp`. The calibration takes about 30 seconds (resting in each of 3 orientations). Temperature monitoring runs in a background thread and logs bias drift. We're ready to use these parameters in the complementary filter.

The next version (v3.2) uses these calibrated values in a complementary filter to produce drift-free pitch and roll estimates—the first real orientation information our robot has ever had.
