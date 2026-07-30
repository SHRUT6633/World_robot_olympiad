# v2.4 — PID for Straight Line

## What changed

The robot can now drive in a straight line. Well, straighter than before. I implemented a PID controller on the Pi side (`pid_straight.py`) that reads heading from the IMU (Inertial Measurement Unit, a BMI270 on the I2C bus) and adjusts the motor speeds to correct heading errors. The steering servo stays centered; the differential motor speed does the correction.

The control loop runs at 50 Hz (every 20 ms):
1. Read gyro heading from the ESP32 (the BMI270 is connected to the ESP32, not the Pi).
2. Compute heading error = target_heading - current_heading.
3. Compute PID output: `u = Kp * error + Ki * integral + Kd * derivative`.
4. Apply correction: `left_speed = base_speed + u`, `right_speed = base_speed - u`.
5. Send left and right speed commands to the ESP32.

The IMU data comes over UART. The Pi sends `{"cmd": "poll_imu"}` and the ESP32 responds with `{"cmd": "imu_data", "gyro_z": 0.5, "heading": 180.2}`. The gyro_z is the yaw rate in degrees/second. Heading is computed by integrating gyro_z on the ESP32 using the Madgwick filter (or simple integration, haven't decided yet).

## Why it changed

The robot doesn't drive straight naturally. Even on a perfectly smooth floor, slight differences in wheel diameter (manufacturing tolerance), motor efficiency, and tire pressure cause the robot to veer. In v2.0 testing, the robot drifted about 30 cm over a 2 m run. That's unacceptable for line following—the camera needs to stay roughly centered over the line, and a 30 cm drift means the line is completely out of frame after 2 m.

## Errors encountered

The integral windup problem appeared immediately. I set up a test: robot on the floor, PID enabled, target heading 0 degrees (straight ahead). I started the robot and let it run. The first 2 seconds were fine—the robot corrected small heading errors with small motor adjustments. Then at t=2.5 seconds, the robot suddenly veered hard right and hit a chair.

Looking at the logs, the heading error at t=2.5s was only 0.5 degrees, but the integral term was at 127 (out of a maximum PID output of ±255). The integral had been accumulating error from the first 2 seconds (where the robot had small but persistent heading errors due to the floor not being perfectly level). When the robot finally corrected those errors, the integral didn't wind down fast enough, so it kept adding correction even when the error was nearly zero. A small disturbance (the robot hitting a slight bump in the floor) caused a brief heading error, and the integral amplified it into a full-scale correction.

The integral windup formula is: `integral = integral + error * dt`. If the error is consistently +2 degrees (robot drifting right), the integral grows linearly with time. After 2 seconds at 50 Hz (100 iterations), integral = 2 * 100 = 200 (scaled by Ki). When the error drops to 0, the integral is still 200, so the PID output is 200, which saturates to 255 (full correction). The robot overcorrects, then the heading error goes negative, and the integral starts winding down from 200—which takes another 2 seconds.

The fix is anti-windup. There are several approaches:
1. **Clamping**: Limit the integral term to a fixed range.
2. **Conditional integration**: Only integrate when the PID output is not saturated.
3. **Back-calculation**: Reduce the integral when the output saturates by feeding back the saturation error.

I implemented clamping as the simplest solution:

```python
self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
```

With `integral_limit = 50` (out of ±255 PID output range), the integral windup is bounded. The robot now corrects smoothly without runaway.

But I didn't stop there. I also added conditional integration: the integral is only updated if the absolute heading error is less than 10 degrees. If the robot is way off (e.g., more than 10 degrees), integral doesn't accumulate—we don't want the integral to build up while the proportional term is doing the heavy lifting.

```python
if abs(error) < 10.0:
    self.integral += error * dt
```

## Alternative approaches considered

1. **Pure proportional control**: No integral term at all. P-only control is simple and stable. But it has steady-state error—the robot will always drift slightly because the proportional term needs a non-zero error to produce a non-zero output. With P-only, I measured about 5 cm of drift over 2 m. Better than 30 cm with no control, but not good enough.

2. **PI with leaky integrator**: Multiply the integral by a decay factor at each step: `integral = integral * 0.99 + error * dt`. This prevents windup and lets the integral decay naturally. I tried `decay = 0.995` and it worked well, but it's more tuning parameters to manage.

3. **Feed-forward**: Use the known motor calibration from v2.2 to compute a feed-forward term that compensates for known motor differences. This is deterministic—if we know the left motor is 5% weaker, we can add 5% to the left speed preemptively. I implemented feed-forward as well, which reduced the PID's workload.

I ended up using all three: feed-forward for the known calibration, P for quick response, and I with clamping for steady-state error correction.

## Reasoning

The PID gains were tuned using the Ziegler-Nichols method. I set Ki and Kd to zero, increased Kp until the robot oscillated (Kp_critical ≈ 12), then set Kp = 0.6 * 12 = 7.2, Ki = 2 * 7.2 / 0.5 = 28.8 (with Ti = 0.5 * oscillation_period). After tuning, the robot maintains heading to within ±1.2 degrees at 0.5 m/s and within ±3.5 degrees at 1.0 m/s. The higher error at speed is due to tire slip—the robot slides sideways slightly during aggressive corrections, which the gyro registers as heading change but the wheels don't actually respond to.

I also added a heading reset function: when the robot starts, the current heading is captured and used as the target heading. This means the robot doesn't need to be perfectly aligned with a global reference at startup. The PID just needs to keep the heading constant, whatever it is.

One implementation detail: the IMU heading is filtered with a complementary filter (gyro + accelerometer) running on the ESP32. The ESP32 sends heading at 100 Hz, but the PID runs at 50 Hz. I added a mutex-protected buffer to ensure the PID always reads the latest heading value without racing with the IMU task.
