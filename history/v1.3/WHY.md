# Version 1.3 — The One-Letter Bug

## The Error

```
NameError: name 'MerkedScaledSigmaPoints' is not defined
```

I stared at this for 10 minutes. What is MerkedScaledSigmaPoints? I have
never seen that word in my life.

Then I looked at the line:

```python
from filterpy.kalman import MerkedScaledSigmaPoints
```

Oh. It is supposed to be **Merwe**ScaledSigmaPoints. Merwe, not Merked.
"R" and "K" are next to each other on the keyboard. I typed Mer-ked
instead of Mer-we.

One letter. The entire UKF (Unscented Kalman Filter) would not initialize.
Without the UKF, the robot does not know where it is. It is blind.

## Why the UKF Matters

The UKF is the brain of the robot's localization system. It takes noisy
data from:
- IMU (accelerometer + gyroscope) — tells us acceleration and rotation
- Magnetometer — tells us which way is north
- Odometry (wheel encoders) — tells us how far we moved

...and combines them into a single best-guess of position (x, y, heading).

Without it, the robot relies on raw sensor data, which is noisy.
With it, the robot has a smooth, filtered estimate.

## Why I Used UKF Instead of Something Simpler

**Option A: Dead reckoning.** Just add up wheel encoder ticks to track
position. Simple, but errors accumulate. After one lap, the robot thinks
it is 50cm away from where it actually is.

**Option B: Complementary filter.** Good for attitude (pitch, roll).
We actually use this too (for the IMU). But it does not handle position.

**Option C: EKF (Extended Kalman Filter).** The standard choice for
robotics. But it linearizes the system (approximates curves as straight
lines), which introduces error in sharp turns.

**Option D: UKF (Unscented Kalman Filter).** What we chose. Instead of
linearizing, it passes "sigma points" through the nonlinear function.
More accurate than EKF for the same computational cost.

The choice was UKF because our robot makes sharp 90° turns where EKF
linearization errors would be significant.

## The Ctrl+C Bug

Same version. Different problem.

When you press Ctrl+C on the Pi, it sends a signal to stop the program.
But our scheduler callbacks had `while True` loops inside them. The
signal could not interrupt an infinite loop.

```python
async def some_task():
    while True:  # BAD — this never stops
        do_something()
```

The fix: remove the while True from every callback. The scheduler already
calls the callback in a loop. The callback just does ONE iteration.

```python
async def some_task():
    do_something()  # One iteration, scheduler calls me again
```

This seems obvious now. But at 3 AM, when you are tired and just want the
robot to work, you add while True without thinking.

## Why This Took 2 Hours

The error message for `MerkedScaledSigmaPoints` was misleading. It said
the module `filterpy.kalman` does not have this attribute. I checked the
module. It has `MerweScaledSigmaPoints`. I thought: "Maybe my filterpy
version is old?" I spent 1 hour upgrading, reinstalling, restarting.

The actual problem: I typed the wrong name. Period.

**Lesson:** Before changing your environment (upgrading packages, changing
settings), read the error message more carefully. The answer is often in
the error. We just read it too fast.

## This Version's Evidence

In `pi/fusion/ukf.py`, line 37:
```python
from filterpy.kalman import MerweScaledSigmaPoints  # Was Merked (typo)
```

The comment is there because I never want to make this mistake again.
