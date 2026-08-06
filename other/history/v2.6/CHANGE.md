# v2.6 — Stop and Reverse

## What changed

The robot can now stop on command and drive in reverse. I wrote `stop_reverse.py` to test emergency stop distances at various speeds, and `dynamic_brake.py` which implements the actual braking logic on the ESP32.

The test procedure in `stop_reverse.py` is:
1. Drive forward at a set speed (10%, 25%, 50%, 75%, 100%).
2. After 2 seconds (or when a stable speed is reached), send the stop command.
3. Measure the coasting distance—how far the robot travels from the stop command until it actually stops.
4. Log the distance for each speed.

The dynamic braking implementation on the ESP32 (`dynamic_brake.py` controls this via UART): when a stop command is received with `brake=true`, the ESP32 briefly reverses the motor polarity (e.g., full reverse for 50 ms) and then sets PWM to 0. This counteracts the motor's back-EMF and brings the robot to a stop much faster than coasting.

## Why it changed

The robot needs to stop precisely. The WRO course has stop zones where the robot must halt within a marked area. If the robot coasts 30 cm past the stop zone, that's a penalty. Precise stopping is also essential for safety—if the robot is about to hit an obstacle, it needs to stop NOW, not after coasting another half meter.

## Errors encountered

The coasting distance shocked me. At full speed (100% PWM, about 1.8 m/s), the robot coasted 32 cm before stopping when I simply set the PWM to 0. This is because the motor's inertia keeps it spinning, and the back-EMF (the motor acting as a generator) isn't dissipated quickly enough.

The first attempt at dynamic braking set the motor to full reverse for 100 ms. The robot stopped in about 10 cm. Success! But then I noticed the robot had jerked backward about 2 cm after stopping. The 100 ms reverse pulse was too aggressive—it didn't just stop the motor, it reversed it slightly. The fix was to reduce the braking pulse to 30 ms, which stopped the robot in 12 cm with no visible reverse movement.

But a second problem appeared: the L298N H-bridge. When I command full reverse, the direction pins swap (both low for reverse), and the PWM is set to 255. This means the full battery voltage (7.4V from the 2S LiPo) is applied across the motor in reverse. The motor windings are essentially shorted through the H-bridge's low-side MOSFETs during the brake, causing a large current spike. The L298N's thermal shutdown kicked in after about 5 aggressive stops:

```
E (12345) gpio: GPIO output level error
```

Wait, that's not the right error. The actual L298N behavior: the H-bridge got hot enough to trigger thermal shutdown after 5 stops. The robot became unresponsive for about 3 seconds until the L298N cooled down.

I added a cooldown period: after each dynamic brake, the robot waits 1 second before the next command. This gives the H-bridge time to dissipate heat. For emergency stops (safety override), the brake is still applied immediately, but a warning is logged.

## Alternative approaches considered

1. **Coast to stop**: Accept the 32 cm coasting distance and design the stop zones with padding. This is the simplest approach but wastes space on the course and looks unprofessional.

2. **Regenerative braking**: Use the motor as a generator and dump the energy into a resistor (or back into the battery). The L298N doesn't support regenerative braking natively, but you can add a braking resistor across the motor terminals. This would be more efficient than dynamic braking (no mechanical shock) but requires extra hardware.

3. **Mechanical brake**: Add a solenoid-actuated brake pad that clamps the wheel shaft. This is common in industrial robots. But it adds weight, current draw, and mechanical complexity.

4. **Ramp-down**: Gradually reduce the PWM to 0 over 200 ms instead of cutting it instantly. This would reduce the coasting distance by a little (the motor would decelerate faster due to back-EMF) but not nearly as much as active braking. I tested this: ramp-down over 200 ms gave a coasting distance of 24 cm. Better than 32 cm, but not good enough.

I went with dynamic braking because it's purely software, costs nothing, and reduces coasting distance from 32 cm to 12 cm. The thermal issue is manageable with the cooldown period.

## Reasoning

The dynamic braking implementation has three modes:
1. **Normal stop** (no brake): Set PWM to 0, let the robot coast. Used when stopping at a known stop zone with plenty of margin.
2. **Soft brake** (brake=50ms reverse): Brief reverse pulse. Used for normal precise stopping (e.g., at a waypoint).
3. **Emergency brake** (brake=200ms reverse): Aggressive reverse pulse for emergency stops. Triggers thermal warning.

The stop distances measured at each speed with soft brake:
- 10% speed: 2 cm coast
- 25% speed: 4 cm coast
- 50% speed: 7 cm coast
- 75% speed: 10 cm coast
- 100% speed: 12 cm coast

These measurements assume the command is sent immediately. In practice, the UART latency adds about 2-5 ms, which at 1.8 m/s adds 3-9 mm of distance. Negligible.

I also added a `stop_and_report()` function in `stop_reverse.py` that sends the stop command and then polls the odometry to confirm the robot has actually stopped. If the odometry still shows movement after 500 ms, an error is raised. This catches cases where the dynamic brake failed (e.g., L298N thermal shutdown) and alerts the operator.

The reverse drive was simpler: just set direction pins to reverse (both low) and send positive PWM. The ramp-up from v2.0 is also applied in reverse. The robot now drives backward at the same speed as forward, with the steering servo working normally (turning in reverse still requires Ackermann compensation, but the same `ackermann.py` code works).
