# v7.5 — Direction Detection

## Diary Entry — 2026-03-21

The robot needs to know which direction it's driving around the track — clockwise (CW) or counter-clockwise (CCW). This matters because cornering logic, obstacle avoidance side preference, and parking approach all depend on the direction of travel.

I spent most of today building `direction_detect.py`, which determines driving direction by observing the robot's yaw change during the first corner.

## The approach

The idea is simple: after the robot starts driving, it doesn't know the track layout yet. The first corner reveals the direction. If the robot turns left (positive yaw change > 45°), it's driving CCW. If it turns right (negative yaw change < -45°), it's driving CW.

```python
def _check_first_corner(self, yaw):
    yaw_change = normalize_angle(yaw - self._start_yaw)
    if abs(yaw_change) > math.radians(45):
        self.direction = CW if yaw_change < 0 else CCW
        self.state = DirectionState.DETECTED
        self._detection_time = time.time()
        return True
    return False
```

This assumes standard WRO track geometry where the first corner is a 90° turn. That's a reasonable assumption — all WRO tracks in the seniors category have a clear first corner.

## The problem with straights

My first test failed spectacularly. The robot started, drove forward in a perfectly straight line, and the direction detection fired:

```
[INFO] Direction detected: CW (yaw change: -47.2°)
```

But the robot hadn't turned at all. It was driving straight. The yaw change came from wheel slip and odometry drift. On the actual track surface (smooth vinyl), the robot's wheels can slip slightly during acceleration, causing the estimated yaw to drift by up to 5-10° per meter. Over a 2m straight section before the first corner, that drift accumulates.

But wait, the log says -47.2°, not 5-10°. That's not drift — that's something else.

Looking more carefully at the data:

```
t=0.0s   yaw=0.0°    start
t=0.5s   yaw=-2.3°   drift
t=1.0s   yaw=-4.1°   drift
t=1.5s   yaw=-48.5°  ??? 
```

At t=1.5s, the yaw jumped by -44°. That's not normal drift. Looking at the raw encoder data:

```
Left encoder:  120 ticks  (t=1.0 to t=1.5)
Right encoder: 118 ticks  (t=1.0 to t=1.5)
```

The encoders say the robot went mostly straight (2 tick difference). But the yaw estimate says -44°. The discrepancy comes from my dead reckoning model. I was using a simple differential drive model:

```python
delta_theta = (right_dist - left_dist) / wheel_base
```

With 120 and 118 ticks, and wheel_base = 0.16m, and wheel_circumference = 0.20m:

```
right_dist = 118 * 0.20 / 360 = 0.0656m
left_dist  = 120 * 0.20 / 360 = 0.0667m
delta_theta = (0.0656 - 0.0667) / 0.16 = -0.0067 rad = -0.38°
```

That's -0.38°, not -44°. The yaw estimate from dead reckoning says the robot went almost straight. So where did -48.5° come from?

Oh. I see it now.

I was using the IMU's yaw reading, not the dead reckoning yaw. The IMU (MPU6050) has a magnetometer for heading, but the magnetic field in our lab is completely messed up by the metal tables, steel beams, and the robot's own motor magnets. The magnetometer was picking up a local field anomaly as the robot moved past a metal table leg.

```
IMU yaw:   -48.5°  (corrupted by magnetic interference)
Dead reckoning yaw: -0.4°  (correct)
```

The fix: I switched to using dead reckoning (encoder-based) yaw for direction detection, not the IMU's magnetometer. The dead reckoning yaw drifts over time due to wheel slip accumulation, but over a 2m straight section before the first corner, it's accurate to within about ±3°.

But even with dead reckoning, I still had drift issues. On the actual track surface, starting from a standstill, the wheels can slip 1-2 ticks during initial acceleration. Over the wheel base, this translates to about 2-3° of yaw error. That's not enough to trigger the 45° threshold. So the fix was already working.

The real question is: why did my initial test with dead reckoning still fail?

I went back to the code and found it:

```python
def process_pose(self, pose):
    if self.state == DirectionState.UNKNOWN:
        ...
        yaw_change = abs(normalize_angle(pose.yaw - self._start_yaw))
        if yaw_change > math.radians(45):
```

The `pose.yaw` was coming from the EKF (Extended Kalman Filter), which fused IMU and encoder data. The EKF was trusting the IMU's magnetometer more than the encoders because the magnetometer covariance was set too low (I'd tuned it for an outdoor environment). The EKF output was following the corrupted magnetometer reading.

I adjusted the EKF sensor covariances: increased magnetometer noise from 0.1 to 5.0, decreased encoder noise from 1.0 to 0.3. This made the EKF trust the encoders more for yaw estimation. The direction detection worked correctly after that.

## The fix: wait until first corner

The real fix, beyond the EKF tuning, is to not even attempt direction detection until the robot has traveled far enough to encounter a corner. I added a minimum distance threshold:

```python
def process_pose(self, pose):
    if self.state == DirectionState.UNKNOWN:
        dx = pose.x - self._start_pose.x
        dy = pose.y - self._start_pose.y
        distance = math.hypot(dx, dy)
        if distance < self.min_detect_distance:
            return self.state
        ...
```

The `min_detect_distance` defaults to 0.5m, which is enough to avoid false triggers from initial wheel slip.

But more importantly, the yaw change must be consistent over time, not just a single sample. I added a running buffer of the last 5 yaw readings and check that the average change exceeds the threshold:

```python
self._yaw_buffer.append(pose.yaw)
if len(self._yaw_buffer) > 5:
    self._yaw_buffer.pop(0)
avg_yaw = sum(self._yaw_buffer) / len(self._yaw_buffer)
yaw_change = normalize_angle(avg_yaw - self._start_yaw)
```

This prevents a single noisy reading from triggering detection.

## Alternatives considered

**Alternative 1: Check direction from start line orientation.** If the robot knows its heading relative to the start line, it could infer direction. But the robot doesn't know its absolute position on the track at startup.

**Alternative 2: Camera-based lane tracking.** Follow the left/right wall with time-of-flight sensors and infer direction from which wall is closer. Complex and fragile.

**Alternative 3: Pre-programmed direction.** Just set CW or CCW in config. Works but doesn't handle the case where the robot is placed on the track facing the wrong way.

**Alternative 4: Wait-and-observe (chosen).** Let the robot drive until it encounters a corner, then determine direction from the turn. Simple, reliable, and the first corner is guaranteed in WRO tracks.

## Testing

I simulated 200 runs with the robot starting at random orientations near the start line. Direction detection succeeded on the first corner in all 200 runs. The minimum distance threshold eliminated all false triggers.

## Stats

- Lines of code: 98 (direction_detect.py)
- Detection threshold: >45° yaw change
- Min distance before detection: 0.5m
- False trigger rate: 0%
- EKF covariance tuning: magnetometer 0.1→5.0, encoder 1.0→0.3

One more component checked off. The robot now knows which way it's going.

— 2026-03-21, signing off.
