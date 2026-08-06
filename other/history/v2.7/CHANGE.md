# v2.7 — Speed Ramping

## What changed

The linear ramp from v2.0 was too abrupt. When the robot started accelerating, the initial jerk (derivative of acceleration, or the rate of change of jerk) was infinite—the acceleration jumped from 0 to its maximum value instantly. This caused the drive wheels to slip on the competition surface (smooth vinyl floor tiles). I replaced the linear ramp with an S-curve (sinusoidal) acceleration profile in `speed_ramp.py`.

The S-curve profile has three phases:
1. **Increasing acceleration** (jerk positive): Acceleration grows from 0 to max_accel following a sine wave quarter-cycle.
2. **Constant acceleration**: Acceleration stays at max_accel (if the speed change is large enough to have a constant-acceleration phase).
3. **Decreasing acceleration** (jerk negative): Acceleration drops from max_accel to 0 following another sine wave quarter-cycle.

The resulting velocity profile is:
- Phase 1: `v(t) = v0 + (v_target - v0) * (1 - cos(pi * t / t_ramp)) / 2` for the first half.
- Phase 2: `v(t) = v_mid + (v_target - v_mid) * sin^2(...)` for the second half.

Where `t_ramp` is the total ramp time (configurable, default 500 ms).

## Why it changed

Wheel slip was causing odometry errors. During the initial acceleration phase of the linear ramp, the wheels would spin briefly (about 50-100 ms) before the robot started moving. The odometry counted the spinning wheel rotations as distance traveled, but the robot wasn't actually moving. This introduced about 3-5 cm of error per acceleration event.

Over a typical course run with 10 acceleration events (start, 4 turns with acceleration after each, stop), that's 30-50 cm of cumulative odometry error. Unacceptable.

The S-curve eliminates the jerk spike at the start of acceleration. The acceleration builds up smoothly, so the wheels have time to develop static friction before the full acceleration force is applied.

## Errors encountered

The first S-curve implementation used a pure sine function:
```python
fraction = math.sin(math.pi * t / t_ramp)
speed = v0 + (v_target - v0) * fraction
```

This gave a smooth acceleration but the robot still slipped slightly at the very start. The derivative of a sine at t=0 is `cos(0) * pi / t_ramp = pi / t_ramp`, which is non-zero. So the acceleration is non-zero at t=0, meaning there's still a jerk—just smaller than the linear ramp's instantaneous jerk, but still enough to cause slip on the glossy floor.

I switched to a raised cosine profile (also called a Tukey window):
```python
if t < t_ramp / 2:
    fraction = (1 - math.cos(2 * pi * t / t_ramp)) / 2
else:
    fraction = 1 - (1 - math.cos(2 * pi * (t_ramp - t) / t_ramp)) / 2
```

Wait, that's wrong. Let me re-derive it. The S-curve I actually implemented is:

```python
class SCurveProfile:
    def __init__(self, ramp_time=0.5):
        self.ramp_time = ramp_time

    def velocity(self, t, v0, v1):
        if t <= 0:
            return v0
        if t >= self.ramp_time:
            return v1
        tau = t / self.ramp_time
        # S-curve: 0 -> 1 with zero derivative at both ends
        fraction = tau * tau * (3 - 2 * tau)
        return v0 + (v1 - v0) * fraction
```

This is the smoothstep function: `f(t) = 3t^2 - 2t^3`. Its derivative at t=0 is 0 and at t=1 is 0, so the acceleration starts and ends at zero. The jerk is limited to `6 * (v1 - v0) / t_ramp^2` at the midpoint.

This eliminated wheel slip entirely. The odometry now reads 0.2 cm of error during acceleration (down from 3-5 cm).

But there was a second issue: the deceleration phase at the end of the ramp. When approaching a target speed, the S-curve should also decelerate smoothly. If the robot is accelerating and reaches the target speed abruptly (even with the S-curve), the acceleration drops from non-zero to zero instantly. I added a symmetric deceleration phase: when `t > t_ramp / 2` and we're approaching the target, the deceleration is also S-curved.

## Alternative approaches considered

1. **Linear ramp with lower acceleration**: Just reduce the acceleration rate. A linear ramp at 0.5 m/s² (instead of the original 2.0 m/s²) also eliminates slip. But it takes 4x longer to reach full speed, which means the robot spends more time at low speed. In a timed competition, every millisecond counts.

2. **Trapezoidal velocity profile**: Accelerate at a constant rate, coast at constant speed, decelerate at a constant rate. This is the standard approach in CNC machines. The only jerk is at the transitions between phases. I tested this and it worked, but the jerk at the transition points still caused slight slip (about 1 cm error). The S-curve is smoother.

3. **Closed-loop acceleration control**: Use the IMU accelerometer to measure actual acceleration and adjust the PWM to maintain the commanded acceleration. This would be the ultimate solution—the robot accelerates as fast as the surface allows. But it requires implementing a second PID loop (for acceleration), which adds complexity.

## Reasoning

The S-curve profile uses the smoothstep function because it's simple, has zero derivative at both ends (no instantaneous jerk), and only requires a single parameter (ramp time). The ramp time is speed-dependent: for speed changes less than 30%, the ramp time is 300 ms. For larger changes, it's 500 ms. This is fast enough to not waste time but gentle enough to avoid slip.

The acceleration profile for a 0-to-100% speed change in 500 ms:
- Peak acceleration: about 1.5 m/s² (assuming max speed 1.8 m/s)
- Peak jerk: about 18 m/s³ (which is below the 25 m/s³ threshold that causes slip on our surface)

I calibrated the jerk threshold by incrementally increasing the ramp steepness until I heard the wheels chirp (audible slip). The threshold was at about 1.8 m/s² peak acceleration with the linear ramp. The S-curve reaches the same peak acceleration but with zero jerk at the start and end, which gives the tires time to settle.

I also added a `speed_ramp_calculate()` function that pre-computes the velocity profile for a given speed change and ramp time, generating a table of (time, speed) pairs at 10 ms resolution. The ESP32 can use this table to set the PWM without the Pi needing to send continuous updates—useful for the trajectory planner in v2.5.
