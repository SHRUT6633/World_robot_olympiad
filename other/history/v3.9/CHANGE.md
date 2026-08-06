# v3.9 — Sensor Health Monitor

## What Changed
We now have 6 sensors running simultaneously: MPU6050 IMU (v3.0-3.2), QMC5883L magnetometer (v3.3), two VL53L0X ToF sensors (v3.4), one VL53L1X ToF sensor (v3.4), and the PiCamera (v3.6). That's a lot of I2C traffic, a lot of potential failure points, and a lot of things that can go wrong during a WRO run.

`sensor_health.py` adds a centralized health monitor that:
1. Tracks read success/failure for each sensor over a rolling window.
2. After 50 consecutive failures, marks the sensor as "disabled" and stops trying to read it.
3. Reports sensor status as a dictionary: `{"imu": "ok", "mag": "ok", "tof_left": "warning", "tof_right": "disabled", "tof_front": "ok", "camera": "ok"}`.
4. Logs errors at a rate-limited frequency (max 1 error per 2 seconds per sensor) to prevent log spam.

The health monitor is a Python class `SensorHealthMonitor` with methods:
- `report_success(sensor_name)` — called when a sensor read succeeds.
- `report_failure(sensor_name, error_msg)` — called when a sensor read fails.
- `status()` — returns current status strings for all sensors.
- `should_read(sensor_name)` — returns False if the sensor is disabled.

## Why
Sensors fail. I2C bus errors happen (we saw them in v3.0). The camera buffer stalls (v3.6). ToF sensors return invalid readings when out of range (v3.4). Without a centralized health monitor, each sensor module has to handle its own error recovery, leading to duplicated error-handling code and inconsistent behavior.

More importantly, if a sensor fails during a competition run, we need to degrade gracefully. If the right ToF sensor fails, the robot should stop trying to read it and rely on the left ToF only. If the IMU fails, we can't compute pitch/roll, but we can still drive using dead reckoning. If the camera fails, we lose pillar detection but can still do wall following with ToF sensors.

The 50-consecutive-failure threshold isn't arbitrary. We measured: at 30 Hz (ToF), 50 failures ≈ 1.7 seconds. At 95 Hz (IMU), 50 failures ≈ 0.5 seconds. This is long enough to tolerate brief I2C glitches but short enough to disable a truly dead sensor before the robot relies on it.

## Errors Encountered

### Logger Prints Every Failure (Spam)
The first version logged every single failure with `print()`. In 30 seconds of testing, the console was flooded with thousands of lines:

```
ERROR: tof_left read failed: OSError(121, 'Remote I/O error')
ERROR: tof_left read failed: OSError(121, 'Remote I/O error')
ERROR: tof_left read failed: OSError(121, 'Remote I/O error')
ERROR: tof_left read failed: OSError(121, 'Remote I/O error')
ERROR: tof_left read failed: OSError(121, 'Remote I/O error')
ERROR: tof_left read failed: OSError(121, 'Remote I/O error')
... (repeated 500+ times in 10 seconds)
```

The log file grew by 10 MB per minute. The repetitive messages made it impossible to find other, more interesting errors in the noise.

```
WARNING: Log file size: 47 MB after 5-minute test
ERROR: Can't find the actual bug because it's buried in 50000 identical log lines
```

**Fix:** Implement rate-limiting. Each sensor has a `last_log_time` dictionary. A failure is only logged if at least 2 seconds have passed since the last logged error for that sensor. The consecutive failure counter still increments (so the 50-consecutive-failure threshold still triggers a disable), but the log message is suppressed.

```python
def log_error(self, sensor, msg):
    now = time.monotonic()
    if now - self._last_log.get(sensor, 0) >= 2.0:
        print(f"ERROR: {sensor}: {msg}")
        self._last_log[sensor] = now
```

After this fix, the log went from 47 MB (5 minutes) to 2.3 KB. Huge improvement.

### Sensors Disabled Prematurely During Startup
On boot, some sensors take time to initialize (the IMU takes 1 second per v3.0 fix, the ToF sensors take ~500 ms to perform their first ranging). During this time, all reads fail. The health monitor counted these as consecutive failures, and by the time the sensors were ready, they were already disabled.

```
ERROR: imu disabled after startup (47 consecutive failures)
ERROR: tof_left disabled after startup (43 consecutive failures)
ERROR: Robot thinks all sensors are dead and refuses to move
```

**Fix:** Add a startup grace period (3 seconds) during which failures are not counted. The grace period starts when `SensorHealthMonitor` is instantiated and ends after 3 seconds. We also allow sensors to be explicitly "registered" with a flag indicating whether they take time to initialize.

```python
self._startup_deadline = time.monotonic() + 3.0

def report_failure(self, sensor, error_msg):
    if time.monotonic() < self._startup_deadline:
        return  # grace period, don't count
    ...
```

### False Disable From One Bad I2C Bus Cycle
A single I2C bus lockup (caused by the MPU6050 holding the SCL line low, which happens occasionally) caused all I2C sensors to report failure simultaneously. Each sensor accumulated 1 consecutive failure. Over a few bus lockups, all sensors hit 50 consecutive failures at the same time.

**Fix:** Increment the consecutive counter only if the previous read also failed. If the previous read succeeded, reset the counter to 0. This means a single bus glitch doesn't contribute to the consecutive count—only sustained failures do.

```python
if not self._prev_success[sensor]:
    self._consecutive_failures[sensor] += 1
else:
    self._consecutive_failures[sensor] = 1  # reset counter, start fresh
self._prev_success[sensor] = False
```

## Alternatives Considered
- **Watchdog timer**: Instead of polling each sensor, use a hardware watchdog that resets the Pi if sensors fail. Too drastic—a single sensor failure shouldn't reset the whole robot.
- **Sensor re-init on failure**: Instead of disabling the sensor, try re-initializing it (reset I2C bus, re-run init sequence). This could recover working sensors. We'll implement this in v4.0; for now, disable is safer than a potentially broken sensor.
- **ROS diagnostics**: The ROS `diagnostic_updater` package provides exactly this functionality (publish diagnostic status at 1 Hz). But we're not using ROS—our code is a single-threaded Python loop.
- **Email/SMS alerts**: Absurd for a robot that runs for 5 minutes per match.

## Current Status
`sensor_health.py` is integrated into the main control loop. All sensor reads go through the health monitor. After 50 consecutive failures, a sensor is disabled and its default value is used (0 mm for ToF, identity quaternion for IMU, last known heading for magnetometer). Log spam is eliminated via rate limiting.

With all 10 versions (v3.0 through v3.9), the SENSING THE WORLD phase is complete. The robot now has: calibrated IMU with drift-free pitch/roll/yaw, wall distance estimation from fused ToF sensors, and color blob detection for pillar identification. Every sensor has been battle-tested against real failure modes. We're ready to move to the next phase: ACTING ON THE WORLD.
