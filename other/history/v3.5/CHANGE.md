# v3.5 — Multi-ToF Fusion

## What Changed
Three Time-of-Flight sensors were reading distance data (v3.4), but each sensor's output was being used individually. For wall following, we need to combine left and right readings into a single "wall proximity" estimate that accounts for the robot's orientation. If the robot is angled toward the left wall, the left sensor reads closer than the right sensor, even if the robot is centered. We need to separate "lateral offset" from "angular offset."

`tof_fusion.py` implements a simple geometric model. The left and right sensors are mounted 160 mm apart (center-to-center). If both sensors read the same distance, the robot is parallel to the walls. If they differ, the robot is angled, and the difference tells us the angle. The front sensor gives forward obstacle detection.

Fusion algorithm:
1. `wall_distance = (left_reading + right_reading) / 2` — approximate distance to side walls
2. `wall_angle = atan2(right_reading - left_reading, SENSOR_SPACING)` — angle of robot relative to walls
3. `front_obstacle = min(front_reading, 4000) / 1000.0` — normalized front distance (0-4 meters, scaled to 0-1 for the control system)

The output is a `WallState` namedtuple with `left_mm`, `right_mm`, `front_mm`, `wall_dist_mm`, and `wall_angle_rad`.

## Why
Raw ToF data is noisy and incomplete. A single sensor can't distinguish between being close to the wall and being angled toward it. By fusing two side sensors, we get a more robust wall estimate that the PID controller can use for wall following. The WRO track has parallel walls in the straight sections, and the robot needs to maintain a consistent distance (typically 150-200 mm from the left wall).

The front sensor fusion is simpler: just smoothing. We apply a moving average filter (window of 5 samples) to reduce noise. The front reading is used for obstacle avoidance—if it drops below 500 mm, the robot stops or turns.

## Errors Encountered

### Crosstalk When Two ToFs Fire Simultaneously
The first integration test showed wild readings. When both the left and right VL53L0X sensors were running, left would report 2000 mm (max range) even when the wall was 200 mm away. Right would report 150 mm correctly. But when we covered the right sensor, left started working correctly again.

```
WARNING: Left ToF: 1987 mm (wall is 200 mm away!)
WARNING: Right ToF: 189 mm (expected 200 mm)
WARNING: Left ToF (right covered): 203 mm (correct!)
```

This is optical crosstalk. Both VL53L0X sensors emit 905 nm laser pulses. When they fire at the same time, the left sensor's photodiode can detect the right sensor's laser pulse reflecting off the common wall. The sensors think the photon took longer to return (farther away) because the path is longer: right emitter → wall → left detector.

**Fix:** Stagger the sensor readings by 20 ms. Read left at t=0, right at t=20ms, front at t=40ms, then repeat. This ensures only one VCSEL is active at any instant. The 20 ms stagger adds 60 ms to the total cycle, reducing the effective rate from 30 Hz to ~16 Hz, but it eliminates crosstalk completely.

```python
def read_all_staggered():
    left_d = read_left()
    time.sleep(0.020)
    right_d = read_right()
    time.sleep(0.020)
    front_d = read_front()
    # Don't sleep after front; next cycle starts immediately
    return left_d, right_d, front_d
```

We also tried moving the sensors farther apart (increasing the mounting angle to widen the beam separation) but the chassis is too small. The electrical engineers suggested shielding the sensors with opaque dividers (black plastic barriers between the sensors). We 3D-printed small baffles that block the direct line of sight between left and right sensors. This helped but didn't fully eliminate crosstalk—the stagger is the real fix.

### VL53L1X Interference With VL53L0X Even When Staggered
Even with 20 ms stagger, the front VL53L1X could interfere with the side VL53L0X sensors. The VL53L1X uses a different modulation frequency (940 nm vs 905 nm), but its photodiode is broadband enough to detect the VL53L0X's IR.

**Fix:** We set the VL53L1X to use its own internal timing generator and ensured the VL53L0X sensors are stopped during VL53L1X measurement. We also used `set_measurement_timing_budget()` to force the sensors to complete their measurement within the 20 ms window.

### Running Average Initialization
When the robot starts, the moving average filter for front distance starts with zeros, so the first 5 readings are biased low. This caused the robot to think there was an obstacle immediately on startup and refuse to move.

```
ERROR: Front obstacle detected at startup! (fused = 0 mm)
ERROR: Robot refuses to start due to false obstacle
```

**Fix:** Initialize the moving average buffer with the first valid reading instead of zeros.

```python
# Before
front_buffer = [0, 0, 0, 0, 0]
# After  
front_buffer = [initial_front_reading] * 5
```

## Alternatives Considered
- **Kalman filter**: A 2-state Kalman filter (lateral distance, wall angle) would be more accurate than the simple geometric model. We prototyped one, but the tuning was fragile—the measurement noise covariance changed with surface reflectivity. The geometric model is simpler and works for the flat white walls of the WRO track.
- **Ultrasonic cross-check**: Using HC-SR04 readings as a sanity check for the ToF data. Not implemented because we're already seeing reliable data after the stagger fix.
- **Single wide-beam sensor**: A single VL53L1X with a diverging lens could measure both sides in one shot. But the beam would be unfocused and inaccurate for wall following.

## Current Status
`tof_fusion.py` outputs fused wall state at ~16 Hz. The data is verified against tape-measured distances: ±15 mm for side walls, ±30 mm for front (longer range, more noise). The PID wall follower (not yet written) will use `wall_dist_mm` and `wall_angle_rad` as inputs.

Next version (v3.6): Camera frame capture for vision-based tasks. The ToF sensors handle proximity, but we need vision to detect colored pillars and goal zones.
