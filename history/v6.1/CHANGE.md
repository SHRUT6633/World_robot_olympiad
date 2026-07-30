## v6.1 — PID Servo Position Control — 2026-07-15

### Summary

Added closed-loop position control for the steering servo using potentiometer feedback. The servo now reads its actual position via an analog potentiometer (0–3.3 V, 10-bit ADC) and adjusts PWM pulse width to reach a target angle. Without derivative damping, the servo would overshoot by 5–8 degrees on fast position commands, causing a visible steering wobble. Adding the D-term eliminated the overshoot.

### What Changed

The steering servo was previously driven by pulse width alone — `servo_pwm.c` on the ESP32 would set a 50 Hz PWM signal and trust that the servo's internal controller reached the commanded position. This works for hobby servos under no load, but our steering linkage has significant friction and return-spring tension. The servo would consistently undershoot by 2–3 degrees during slow movements and oscillate around the target after fast movements. The undershoot was bad enough that the Stanley controller's cross-track error budget was consumed entirely by servo inaccuracy.

I built `servo_pid.py` running on the Raspberry Pi. It reads the servo's actual angle via analog input (the potentiometer voltage is sampled by an ADS1115 ADC at 100 Hz over I2C), computes the error relative to the commanded angle, and adjusts the PWM pulse width to drive the error to zero. The I2C read takes about 1.2 ms per sample (ADS1115 at 128 SPS with the PGA set to ±4.096 V). The total control loop is about 3 ms, well within the 10 ms budget for a 100 Hz controller.

### Error: Overshoot on Fast Moves

The first version was a pure PI controller (kp=1.5, ki=0.3, no kd). I commanded a 20-degree step. The servo responded in about 150 ms but overshot by 5.2 degrees, then took another 400 ms to settle. This is unacceptable for the control loop — if Stanley requests a steering angle and the servo takes 550 ms to reach it, the robot is driving with stale steering commands for half a second.

ADC log from a 20-degree step:

```
[100] target=20.0 current=0.0 err=20.0 P=30.0 I=0.0 out=30.0 pulse=1890
[110] target=20.0 current=8.2 err=11.8 P=17.7 I=0.4 out=18.1 pulse=1735
[120] target=20.0 current=15.1 err=4.9 P=7.4 I=0.5 out=7.9 pulse=1587
[130] target=20.0 current=19.3 err=0.7 P=1.0 I=0.6 out=1.6 pulse=1510
[140] target=20.0 current=24.8 err=-4.8 P=-7.2 I=0.5 out=-6.7 pulse=1455
[150] target=20.0 current=17.2 err=2.8 P=4.2 I=0.4 out=4.6 pulse=1545
[160] target=20.0 current=21.5 err=-1.5 P=-2.2 I=0.4 out=-1.8 pulse=1482
```

The servo reached 19.3° at t=130 ms, but the momentum carried it past to 24.8° at t=140 ms. Without a derivative term, the controller didn't "see" the velocity until it was already past the target and the error changed sign. The peak overshoot was 5.2°, and the 1° settling time (error within ±1°) was 550 ms. During those 550 ms, the robot was driving with a steering angle that was up to 25% off from the commanded value.

I also tested a 30-degree step (the mechanical limit). The overshoot was 7.8 degrees, and the servo briefly hit the mechanical stop with an audible thud. The mechanical stop hitting repeatedly would eventually damage the steering linkage, so this had to be fixed urgently.

### Alternatives Considered

1. **Reduce P-gain** — Dropping kp from 1.5 to 0.8 eliminated overshoot but the rise time went from 150 ms to 450 ms. The servo felt sluggish and a Stanley step input would take too long to track. At kp=0.5, the rise time was 720 ms — completely unacceptable. I plotted the whole P-gain sweep: kp vs. overshoot vs. rise time. The tradeoff curve showed that no single P-gain could simultaneously give <150 ms rise time and <2° overshoot. A derivative term was the only way to break the tradeoff.

2. **Low-pass filter on command** — I could rate-limit the servo commands so the servo never receives a large step. I tried a slew rate limit of 100°/s: the 20° step became a 200 ms ramp. This eliminated overshoot (0.8°) but the ramp delay meant the servo lagged the command by 100 ms at all times. For a 50 Hz control loop, 100 ms of phase lag is significant — it would make the robot's steering response feel delayed and could cause instability in the outer Stanley loop.

3. **Add D-term (chosen)** — The derivative term reacts to the rate of change of error. As the servo approaches the target, the error velocity increases in the negative direction, and the D term generates a counteracting force that slows the approach. It's exactly what was needed. The formula: `output = kp * error + ki * integral + kd * (error - last_error) / dt`. The D term acts like a virtual damper, opposing rapid changes in error.

### The Fix

I added kd=0.3 to the servo PID controller. With the D-term active, the 20-degree step overshoot dropped from 5.2° to 0.8° and settling time from 550 ms to 200 ms. The step response was crisp: 90% rise in 140 ms, peak at 20.8°, settle to ±1° in 200 ms. The mechanical stop thud was gone in the 30° test.

I also added a hardware consideration: the analog ADC reading has about ±0.5° of noise (50 mV ripple on the 3.3 V rail from the servo's current draw). The derivative term amplifies noise — with kd=0.3, the raw derivative had a standard deviation of 3.2°/s when the servo was stationary, causing 1.5° of output jitter. I added a first-order low-pass filter (alpha=0.3) on the position reading before computing the derivative:

```python
current_filtered = 0.3 * current_raw + 0.7 * current_filtered_prev
```

This reduced derivative noise by a factor of 5. The output jitter dropped to 0.3°. The 100 Hz response was fast enough that the filter's 3-sample time constant (30 ms) didn't add noticeable lag.

I also verified that the ADS1115 I2C communication was reliable. At 100 Hz polling, the I2C bus had no collisions with the other sensors (IMU and ToF are on the same bus). I used a bus monitor to check: the ADS1115 reads took 1.2 ms each, and the bus utilization was about 15%.

### Remaining Issues

- The servo still has about 0.5° steady-state error due to potentiometer quantization (10-bit ADC = 3.3 mV / count with PGA at ±4.096 V gives about 0.18° per count). This is acceptable for WRO requirements (±1° is the spec).
- The D-term gain (kd=0.3) might need adjusting for different servo loads (e.g., after a crash that bends the linkage). I should probably make it configurable.
- No feedforward term yet — the servo position depends on PWM pulse width, but the mapping is nonlinear (the servo's internal potentiometer is not perfectly linear with angle). Feedforward would help, but I'll save that for v6.3.
- The low-pass filter alpha was chosen by trial and error. I should characterize the noise spectrum with an FFT to set the cutoff frequency scientifically.

### Files

- `servo_pid.py` — PD servo position controller with ADC feedback and low-pass filter
- `servo_step_test.py` — Script that commands servo steps and logs response for tuning
