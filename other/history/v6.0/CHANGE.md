## v6.0 — PID Speed Control — 2026-07-14

### Summary

Implemented closed-loop speed control for the drive motor using a PI controller with encoder feedback. The motor now reads a quadrature encoder at 100 Hz and adjusts PWM duty cycle to match a target speed. The fundamental challenge was oscillation: at low target speeds the motor would stutter and surge repeatedly. The fix was gain scheduling — reducing the proportional gain when the target speed is low.

### What Changed

The motor was previously driven open-loop. `main.py` would write a PWM value (0–255) directly to the ESP32 UART command, and whatever speed came out was what you got. That was fine for straight-line test runs, but as soon as we introduced curvature-based speed variation (slowing for corners), the open-loop mapping became useless. The same PWM value gave different speeds depending on battery voltage, motor temperature, and surface friction.

I added `motor_pid.py` that wraps a PI controller around encoder feedback. The controller runs in velocity form — it computes an increment to the current PWM command rather than an absolute value — so acceleration and deceleration are smooth by construction. The loop runs at 100 Hz (matching the encoder read rate) and the integrator eliminates steady-state error from friction and slope.

### Error: Low-Speed Oscillation

The first test was a disaster. I set target speed to 0.3 m/s, the robot lurched forward, stopped, lurched forward, stopped — a violent oscillation at about 2 Hz. The encoder log showed the speed swinging between 0.0 and 0.6 m/s. The P-gain was 1.2, which worked fine at 1.0 m/s but was clearly too aggressive at low speed.

I added debug logging to capture the exact sequence:

```
[PID] target=0.30 current=0.00 error=0.30 P_term=0.36 I_term=0.00 output=0.36
[PID] target=0.30 current=0.55 error=-0.25 P_term=-0.30 I_term=0.05 output=-0.25
[PID] target=0.30 current=0.15 error=0.15 P_term=0.18 I_term=0.02 output=0.20
[PID] target=0.30 current=0.48 error=-0.18 P_term=-0.22 I_term=0.03 output=-0.19
```

The pattern was clear: at low speed, a small PWM change produced a large speed change relative to the target, so the P term kept overcorrecting. The motor's response at low PWM was nonlinear — around 10–20% duty cycle, the torque-to-speed relationship is much steeper than at 50–80% duty cycle. I characterized this by running an open-loop calibration: I logged the speed at each PWM value from 30 to 255 in steps of 10. The speed per PWM unit at low PWM (0–50) was about 0.008 m/s per PWM count, while at mid-range (100–200) it was about 0.004 m/s per PWM count. So the low-speed sensitivity is double the mid-speed sensitivity, which directly explains why a fixed gain oscillates at low speed.

I confirmed my hypothesis by temporarily switching to pure proportional control (ki=0) and sweeping kp values. At kp=0.3, the oscillation at 0.3 m/s was gone, but the robot took 4 seconds to reach 2.0 m/s. At kp=1.2, the 2.0 m/s step response was crisp (400 ms rise time) but 0.3 m/s oscillated. This told me the plant gain varies by a factor of about 3 across the operating range, which is too much for a fixed-gain controller.

### Alternatives Considered

1. **Lower fixed P-gain** — Dropping kp from 1.2 to 0.4 eliminated the oscillation at 0.3 m/s, but at 2.0 m/s the robot felt sluggish and took 3+ seconds to reach target speed. The motor has different dynamics at different operating points, so a single gain cannot cover the full range. I timed it: at kp=0.4, the 2.0 m/s step had a rise time of 2.8 seconds and the steady-state error due to friction was 0.12 m/s that the integral term took another 1.5 seconds to eliminate. Total settling time: 4.3 seconds. Completely unacceptable for a competitive robot.

2. **Integrator windup guard** — The integrator wasn't the problem here (the oscillation was purely P-driven), but I added a basic clamp anyway — max integral contribution capped at ±50 PWM — since it's a known issue when the motor saturates at 255 PWM. This didn't help the oscillation at all, as expected, but it's cheap insurance.

3. **Adaptive gain via scheduling (chosen)** — I implemented `_select_gains()` that returns a different kp based on the current operating speed. The logic maps the speed to a kp value using breakpoints determined by the open-loop calibration. At very low speed (<0.3 m/s), the plant gain is highest, so kp=0.3. At medium speed (0.3–1.0 m/s), kp=0.6. At high speed (1.0–1.5 m/s), kp=0.9. At full speed (>1.5 m/s), kp=1.0. The transitions are smooth because the speed changes continuously.

### The Fix

The gain schedule is a piecewise-linear map from speed to kp:

```python
if target < 0.3:   kp = 0.3
elif target < 0.8: kp = 0.6
elif target < 1.5: kp = 0.9
else:               kp = 1.0
```

Below 0.3 m/s the gain drops to 0.3; above 1.5 m/s it rises to 1.0. The transition is stepwise between breakpoints. This eliminated the oscillation entirely while maintaining crisp response at high speed (rise time to 2.0 m/s is 620 ms).

I also added a deadband: if the absolute speed error is below 0.02 m/s, the PID output is frozen. This prevents the motor from continuously dithering when it's essentially on target. The dithering before the deadband was about ±0.01 m/s at 50 Hz, which caused an audible high-frequency whine from the motor driver. The deadband eliminated the whine completely.

The calibration script `pid_calibrate.py` runs an open-loop PWM sweep and logs the resulting speed. This gave me the data I needed to set the gain breakpoints. I ran it 3 times (different battery voltages) and averaged the results. The mapping was stable within ±5% across runs, which gave me confidence in the approach.

### Remaining Issues

- The integrator can still wind up if the robot is held stationary while the controller demands speed. I'll address this in v6.5 with proper anti-windup.
- The gain schedule was tuned experimentally for our specific motor and chassis. It will need retuning if we change gearing or wheel diameter. The current gearing is 10:1 with 65mm wheels.
- No acceleration limit yet — if the target jumps from 0 to 2.0 m/s, the PID responds instantly, which could cause wheel slip on smooth surfaces. That's a v6.8 problem where I'll add the velocity profiler.

### Files

- `motor_pid.py` — PI speed controller with encoder feedback and gain scheduling
- `pid_calibrate.py` — Calibration script that measures open-loop PWM→speed mapping
