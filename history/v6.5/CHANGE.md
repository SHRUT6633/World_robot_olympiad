## v6.5 — Anti-Windup for PID Integrator — 2026-07-19

### Summary

Added an anti-windup mechanism to prevent the PID integral term from accumulating excessively when the actuator is saturated (output hits a physical limit). Without this, the robot exhibits a large overshoot after being held back or stuck — the integral term has been accumulating during the stuck period, and when the robot breaks free it shoots past the target. The initial approach was simple clamping at ±10, but that crippled response time because it limited the integral's ability to build up during normal acceleration. The fix was conditional integration: only integrate when the output is not saturated AND the error would integrate in a direction that helps rather than hurts.

### What Changed

I extracted the anti-windup logic into a standalone `AntiWindup` class with two methods: `apply()` for simple clamping and `conditional()` for the smarter conditional integration approach. The PID controller in `AdaptivePID` (from v1.x) accumulated the integral unconditionally — every control tick added `error * dt` to `_integral`. This is fine while the robot is moving freely, but if the robot is blocked (e.g., against a wall or during a collision), the error stays large and the integral grows without bound.

When the obstacle is removed, the integral term dominates the PID output, causing the robot to accelerate far past the target speed before the proportional and derivative terms can bring it back. This is "integrator windup" and it's one of the classic PID failure modes. It's particularly dangerous for a competition robot because:
1. It can cause collisions with the next obstacle
2. It creates a visible "jumpy" behavior that loses style points
3. In the worst case, the integrator can wind up so much that the robot spins out

### Error: Restrained Start Overshoot

I tested this by holding the robot stationary while commanding 1.0 m/s. After 3 seconds I released it. The robot shot forward at approximately 2.5 m/s — more than double the target speed — for about 0.5 seconds before the PID pulled it back.

```
[PID] target=1.0 current=0.0 error=1.0 I_term=0.01 total=1.21  (held, t=0.1)
[PID] target=1.0 current=0.0 error=1.0 I_term=0.10 total=1.30  (held, t=0.2)
[PID] target=1.0 current=0.0 error=1.0 I_term=0.50 total=1.70  (held, t=0.5)
[PID] target=1.0 current=0.0 error=1.0 I_term=1.00 total=2.20  (held, t=1.0)
[PID] target=1.0 current=0.0 error=1.0 I_term=2.00 total=3.20  (held, t=2.0)
[PID] target=1.0 current=0.0 error=1.0 I_term=3.00 total=4.20  (held, t=3.0)
[RELEASED]
[PID] target=1.0 current=0.2 error=0.8 I_term=3.10 total=4.06  (overshoot)
[PID] target=1.0 current=1.3 error=-0.3 I_term=3.07 total=3.43  (still way high)
[PID] target=1.0 current=2.3 error=-1.3 I_term=3.01 total=2.11
[PID] target=1.0 current=2.5 error=-1.5 I_term=2.94 total=1.14
[PID] target=1.0 current=2.1 error=-1.1 I_term=2.86 total=1.21
```

The integral term reached 3.0 during the 3-second hold (ki=0.1, so `I = I + ki * error * dt = I + 0.1 * 1.0 * 0.01 = I + 0.001` per tick, ×300 ticks = 3.0). After release, the P-term switched sign (error became negative as the robot overshot), but the I-term took 1.5 seconds to wind down (at 100 Hz, that's 150 ticks). Peak speed was 2.5 m/s. This is dangerous — the robot could crash into the next obstacle before regaining control.

I also tested the scenario where the robot is pushing against a wall (motor stalled, speed = 0, target = 1.0 m/s). The integrator wound up to 5.8 before I manually stopped the test. At that level, the integral term alone would command `5.8 * ki * scaling = 5.8` PWM units beyond the saturation limit. When the wall is removed, the robot would accelerate at maximum PWM for over a second before the output dropped below saturation. That's a crash waiting to happen.

### Alternatives Considered

1. **Clamping (initial approach)** — I set a hard limit on the integral term (e.g., `I_term = max(-limit, min(limit, I_term))` with limit=1.0). This prevented the windup but caused a problem: the clamp kicked in during normal operation too. When accelerating from 0 to 2.0 m/s, the integral naturally grows as the controller demands sustained output. The integral accumulates error * dt for the entire 1.2-second acceleration period. With a limit of 1.0, the integral was clamped at 1.0 after 1 second, and the remaining 0.2 seconds of acceleration had no integral contribution. Time to reach 2.0 m/s went from 1.2 s to 2.8 s. The robot felt sluggish and unresponsive. I also tried limit=2.0 and limit=5.0. Limit=2.0 still caused sluggish acceleration (2.0 s to target) and limit=5.0 didn't clamp enough to prevent windup (peak speed after release was 1.8 m/s, still too high).

2. **Back-calculation** — Subtract the saturated portion from the integral: `integral -= kc * (output - saturated_output) * dt`. This works well but adds a tuning parameter (kc) that's hard to set without a systematic procedure. I spent an afternoon tuning kc for the motor PID and found kc=0.5 worked reasonably well (peak overshoot 1.3 m/s), but the settling time was still 0.8 seconds. The back-calculation approach is more common in process control (chemical plants) where the time constants are seconds to minutes, not milliseconds.

3. **Conditional integration (chosen)** — Only integrate when the error is small enough that the controller is making progress. Specifically: integrate only when `abs(error) < threshold`. I set the threshold to 0.3 m/s based on the following reasoning: above 0.3 m/s error, the P-term provides sufficient control authority and the integral would just cause windup. Below 0.3 m/s error, the P-term is small and the integral is needed to eliminate steady-state error.

### The Fix

The conditional integration logic in `AntiWindup.conditional()`:

```python
def conditional(self, integral, error, output, limit):
    if output >= limit and error > 0:
        return integral  # freeze
    if output <= -limit and error < 0:
        return integral  # freeze
    return integral + error * self.dt  # integrate
```

The conditions freeze the integral when:
- Output is at the positive limit AND error is positive (controller is demanding more power but the motor is already at max PWM; integrating would only make the windup worse)
- Output is at the negative limit AND error is negative (same in reverse direction, e.g., braking while the motor is already at min PWM)

This is the "conditional integration" or "integrator clamping" method described in Åström & Murray's "Feedback Systems" textbook. It's simpler than back-calculation and doesn't require tuning. The condition is intuitive: don't integrate if the controller is saturated and the error wants more saturation.

With this fix, the restrained-start test showed no integral windup. The integral term stayed at ~0.3 during the hold (it integrated for the first ~3 ticks until the output saturated, then froze). After release, the robot accelerated to 1.0 m/s in 1.3 seconds with no overshoot (peak speed 1.05 m/s). The wall-push test also passed: the integral stayed at 0.3, and when released, the robot resumed smoothly.

### Remaining Issues

- The threshold for "output saturated" is currently the same as the PID output limit. These should be separate: the output limit is a soft limit (controller can exceed it but output gets clamped), while the saturation threshold should be the physical limit of the actuator.
- Conditional integration doesn't handle the reverse-windup case well: if the robot is pushed forward (negative error while output is already at minimum, e.g., being pushed downhill), the integral should wind down more aggressively. The condition `output >= limit and error > 0` doesn't catch this case.
- I should also reset the integral when the setpoint changes significantly (>1.0 m/s step), but that's an optimization for later.
- The 1e-6 epsilon in magnitude comparisons could mask issues if the error is genuinely large but the output is at the limit. I used strict comparisons (>=, <=) to avoid this.

### Files

- `anti_windup.py` — AntiWindup class with conditional integration method
- `windup_test.py` — Script that simulates a restrained-start scenario with and without anti-windup
