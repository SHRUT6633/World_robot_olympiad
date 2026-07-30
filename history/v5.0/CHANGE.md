# v5.0 — Dead Reckoning

**Theme:** "We need to know where we are."

The robot has motors and encoders. That means we can track position. In theory. In practice, dead reckoning is a cruel joke that physics plays on roboticists. But we had to start somewhere.

I wrote `dead_reckon.py` as a simple class that subscribes to encoder tick counts and integrates them into an x,y position using a differential drive model. The math is straightforward: left and right wheel encoder deltas → forward distance and heading change → update pose.

The first test was a straight line of 2 meters. I placed tape marks on the floor at 0.5m intervals. The robot drove forward while I logged the estimated position.

At 0.5m real distance, the estimate was 0.53m — acceptable.
At 1.0m real distance, the estimate was 1.05m — still okay.
At 2.0m real distance, the estimate was 2.21m — that's 21cm of error.

But the real nightmare was turns. I commanded a 90-degree left turn. The encoder-based heading said 92°. Not terrible individually. But then I drove another 1m forward. The lateral error from that 2° heading error multiplied: `sin(2°) * 1m = 3.5cm` lateral drift. After a full lap of four turns, closing the loop, the robot thought it was 34cm away from where it actually was.

The error compounds quadratically because heading errors integrate into position errors that grow linearly with distance traveled, and position errors themselves accumulate. After 1m of travel, error was ~5cm. After 2m, ~20cm. The relationship is approximately `error ~= 5 * d^2` where d is in meters.

I tried calibrating the wheel diameters more precisely. I measured 10 rotations, divided by ticks per rotation, and got wheel circumference to 0.1mm precision. It helped maybe 10%. The fundamental issue is that tiny errors in heading estimation (even 0.5°) cause unbounded lateral position error. Encoder slip, uneven floor, slight tire pressure differences — all of these produce systematic errors that dead reckoning cannot correct.

The fix? Accept the limitation. Dead reckoning is only usable for short distances under 1 meter. Beyond that, the error budget is blown. I added a `distance_traveled` tracker and a warning when cumulative path length exceeds 1.0m. The motion planner will need to reset position estimates frequently using external sensors.

One thing I did get right: the encoder tick processing uses a rolling buffer to debounce edge cases where encoders report spurious ticks during motor startup. That was a lesson learned from v4.x where we had encoder glitches on the first few milliseconds of motion.

The code is clean and tested, but it's a dead end architecturally. Pure dead reckoning cannot sustain a WRO course that requires 5+ meters of navigation with <5cm accuracy. We need absolute position references. That's the next step.

Key files:
- `dead_reckon.py` — The dead reckoning pose estimator
- `motion_tracker.py` — helper that logs and warns on distance thresholds

Error messages and console output from testing:
```
[DEAD_RECKON] Starting 2m straight test...
[DEAD_RECKON] Segment 0.5m: est=0.53m err=3cm
[DEAD_RECKON] Segment 1.0m: est=1.05m err=5cm
[DEAD_RECKON] Segment 1.5m: est=1.61m err=11cm
[DEAD_RECKON] Segment 2.0m: est=2.21m err=21cm ← FAIL
[WARNING] Cumulative path 1.05m exceeds 1.0m threshold. Position uncertainty: HIGH
```

The `[WARNING]` line is the new guard I added. It fires every time the total distance traveled since last reset exceeds 1.0m. The planner will use this as a signal to request a sensor-based correction.

Lessons for the team: Dead reckoning is not a localization strategy. It's a local motion integration strategy. Use it to interpolate between absolute measurements, never to replace them. Going forward, v5.1 will add magnetometer-based heading to at least fix the yaw drift problem. With absolute heading, the dominant error source (heading drift) is eliminated, leaving only odometry scale errors which grow linearly rather than quadratically.
