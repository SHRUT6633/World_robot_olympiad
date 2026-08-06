# v3.4 — ToF Distance Reading

## What Changed
Two Time-of-Flight distance sensors were added to the robot. The VL53L0X (short-range, up to 2 meters) was mounted on each side of the chassis, pointing left and right for wall tracking. The VL53L1X (long-range, up to 4 meters) was mounted on the front for forward obstacle detection. Both use the I2C bus with different addresses (0x29 default for VL53L0X, 0x30 for VL53L1X via XSHUT pin toggling).

`read_tof.py` initializes both sensors, handles the STMicroelectronics VL53L0X and VL53L1X drivers, and reads distance measurements at 30 Hz (VL53L0X) and 50 Hz (VL53L1X). The readings are output in millimeters as integers, with timestamps. The sensors use Class 1 VCSEL lasers (905 nm, eye-safe) and measure time-of-flight of reflected photons to determine distance.

We chose ToF over ultrasonic (HC-SR04) because:
1. ToF has a narrower beam (25° vs 60° for ultrasonic), giving more precise directionality.
2. ToF works on black surfaces (ultrasonic absorbs into foam/soft surfaces).
3. ToF is faster (30-50 Hz vs 10-20 Hz for reliable ultrasonic ranging).

## Why
The WRO 2026 track has walls that the robot must follow and obstacles it must avoid (pillars, ramps, other robots). Without distance sensing, the robot would have no collision avoidance. The stereo configuration (left/right/front) gives us enough information to implement a simple wall-following behavior in the next version.

Mounting positions:
- Left VL53L0X: 80 mm above ground, pointing left at 90° to forward axis
- Right VL53L0X: 80 mm above ground, pointing right at 90° to forward axis
- Front VL53L1X: 50 mm above ground, pointing forward, slightly angled up (5°) to avoid reading the floor

## Errors Encountered

### Sensors Return 0 When Out of Range
The very first test: we pointed the front sensor at a wall 3 meters away (within VL53L1X's 4-meter range). It read 0 mm. We moved it closer—1 meter away—and it read 1002 mm. Moved it back to 3 meters—0 mm again. We thought the sensor was defective.

```
WARNING: Front ToF reading: 0 mm (suspicious)
WARNING: Left ToF reading: 0 mm (expected 450 mm, wall is 45 cm away)
ERROR: Obstacle detection fails — robot drives into wall
```

After reading the datasheet more carefully: the VL53L0X returns 0 or 65535 when the target is out of range or no target is detected. The VL53L1X returns 0 when the measurement is invalid (range status = 0). The status register (0x05 for VL53L1X) indicates the measurement validity.

**Fix:** We added range checking. VL53L0X readings > 2000 mm are clamped to 2000 mm. VL53L1X readings > 4000 mm are clamped to 4000 mm. Readings of 0 or > max range are treated as "no detection" and return None instead of 0. The calling code handles None by assuming the previous valid reading (hold last value).

```python
if distance == 0 or distance > MAX_RANGE:
    return None  # no valid reading
```

We also set the VL53L1X's distance mode to LONG (register: 0x03, VL53L1X_DISTANCEMODE_LONG = 0x02) to ensure it uses the 4-meter timing budget instead of the default 1.3-meter short mode.

### VL53L0X I2C Address Conflict
Both VL53L0X and VL53L1X default to address 0x29. When we connected both sensors, the I2C bus had a conflict—reading returned garbage.

```
ERROR: OSError: [Errno 121] Remote I/O error on bus 1
ERROR: No ACK from I2C address 0x29
```

**Fix:** We use the XSHUT pins to control each sensor's power state. Hold all sensors in reset (XSHUT low), then enable one at a time, set a unique I2C address using `set_device_address()`, then enable the next. The VL53L0X left gets 0x30, right gets 0x31, and VL53L1X gets 0x32 (or stays at 0x29 if it's the only one).

```python
# Address assignment sequence
set_xshut(ALL_SENSORS, LOW)  # all reset
set_xshut(LEFT_TOF, HIGH)    # enable left
left.set_address(0x30)
set_xshut(RIGHT_TOF, HIGH)   # enable right
right.set_address(0x31)
set_xshut(FRONT_TOF, HIGH)   # enable front
front.set_address(0x32)
```

### Timing Budget Too Short For 50 Hz
VL53L1X at 50 Hz requires a timing budget of 20 ms. But with the default measurement budget of 15 ms (for 60 Hz), the sensor occasionally returned invalid measurements (range_status = 4, "wrap around" in the datasheet). The returned distance would randomly jump from 1000 mm to 50 mm.

**Fix:** We set the timing budget to 33 ms (30 Hz) for both sensors, which gives more reliable readings. 30 Hz is sufficient for wall following at 0.5 m/s—the robot travels only 1.7 cm between readings.

## Alternatives Considered
- **Ultrasonic HC-SR04**: Cheaper ($1 each), wider beam (which we don't want), slower, affected by air temperature and humidity. Not accurate on non-perpendicular surfaces.
- **TF-Luna LiDAR**: Single-point ToF with 8-meter range. More accurate but $30 each, and we'd need 3 = $90. Over budget.
- **Sharp IR GP2Y0A02YK**: Analog distance sensor, 20-150 cm range. Lower resolution (8-bit ADC), affected by ambient light, and the output is non-linear (inverse of distance). Would require calibration curve.
- **Kinect/Depth camera**: Overkill and too bulky for the robot chassis.

## Current Status
`read_tof.py` provides left, right, and front distance readings at 30 Hz. Invalid readings are filtered out and replaced with the last valid reading. Range clamping prevents absurd values. The sensors are mounted and wired with XSHUT control for address assignment. Next step: fuse the three ToF readings into wall proximity estimates for wall following (v3.5).
