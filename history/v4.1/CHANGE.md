# v4.1 — Wall Detection from ToF

## What I Tried

The WRO 2026 track has walls around the perimeter. We need to know how far the robot is from the left and right walls and at what angle. I'm using two VL53L1X Time-of-Flight sensors mounted on the sides of the robot.

The idea was simple: read both sensors at 30 Hz, convert raw distance readings into a (distance, angle) pair relative to the robot's centreline.

For a pair of readings `left_dist` and `right_dist`, the wall angle is:

```python
wall_angle = atan2(right_dist - left_dist, sensor_separation)
```

On paper this is straightforward trigonometry. The sensors were configured with the default API settings from the STM `VL53L1X` library.

## The Error — ToF Blind Spot

The first test run failed immediately. The robot was driving along and suddenly reported `left_wall = 65535 mm` (the sensor's max-range error value) and the control loop locked up because the angle calculation returned `NaN`.

After instrumenting the sensor driver, I narrowed it down:

```
[SENSOR] [0.312] [tof_left]: Reading: 28 mm
[SENSOR] [0.313] [tof_left]: Status: 4 (RANGE_STATUS_SIGMA_FAIL)
[WARN] [0.314] [wall_detect]: left_dist = 65535, discarding
```

The VL53L1X has a physical blind spot below ~30 mm. When the robot gets too close to the wall, the returned pulse arrives before the internal calibration window closes, and the sensor either returns `0` or `65535` (depending on firmware version) with a sigma fail status.

This kept happening whenever the robot corrected towards the wall — the correction would make it *too close*, the sensor would blind out, and the control loop had no data.

## What I Changed

I added a minimum-distance check that treats any reading below 35 mm as "wall contact" (distance = 0):

```python
def read_tof(sensor):
    sensor.start_ranging()
    dist = sensor.get_distance()
    status = sensor.get_range_status()
    sensor.stop_ranging()

    if status != 0 or dist < 35:
        return 0.0
    return float(dist)
```

Returning `0` instead of discarding the reading is intentional: if the sensor says we're that close, we're essentially touching the wall, and the controller should steer hard away. Discarding it would leave the last valid reading in the buffer, which could be 100 mm or more, and the robot would keep steering towards the wall because it thinks there's room.

I also added a software timeout — if both sensors return 0 for more than 10 consecutive frames, we assume the robot is wedged and issue an emergency reverse.

## Alternatives Considered

- **Using SHARP IR sensors instead**: IR sensors are less accurate (±10 mm vs ±1 mm for ToF) but don't have the blind spot. However, they're affected by ambient light and the competition hall has bright arena lighting. Stick with ToF.
- **Mechanical bumpers**: Already have bumpers for collision detection, but they only trigger on contact. We need proximity data for path planning.
- **Mounting sensors at an angle**: Angling the sensors forward by 15° would move the blind spot further from the robot, but would also reduce accuracy for perpendicular wall distance measurements.

## Still Broken

- **Cross-talk between sensors**: When both VL53L1X sensors fire at the same time, their IR beams interfere. I've staggered the timing (left reads first, right reads 10 ms later), but I still see occasional ghost readings.
- **Temperature drift**: The ToF sensor's internal VCSEL wavelength drifts with temperature. In a cold start vs. after 10 minutes of runtime, the readings shift by about 3-4 mm. Not critical for wall avoidance, but annoying.

## Lesson Learned

Always check the datasheet for sensor limitations *before* writing the driver. The VL53L1X blind spot is documented on page 12 of the datasheet, but I skimmed it and assumed "30 mm minimum" meant "works down to 30 mm", not "returns garbage below 30 mm". Reading the fine print would have saved me two hours of debugging.
