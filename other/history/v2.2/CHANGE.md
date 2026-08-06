# v2.2 — Speed Control with PWM

## What changed

Up until now, the speed command from the Pi was a percentage (0-100) that the ESP32 mapped to a PWM duty cycle in a hardcoded, one-size-fits-all way. This version formalizes that mapping into a proper speed control module. I wrote `speed_control.c` on the ESP32 side, which takes a speed percentage and maps it to a PWM duty cycle with configurable resolution and frequency.

The mapping is linear: `duty = (speed * max_duty) / 100`. But I added two important features:
1. **Dead zone**: Speeds below 10% are treated as 0. The motor doesn't produce enough torque below about 8% duty to overcome static friction, and sending a 5% signal just wastes power as heat in the H-bridge.
2. **Soft limits**: The maximum duty is capped at 90% of full range (duty 230 out of 255). This provides headroom for the PID controller in v2.4—if the PID needs to increase speed to correct a heading error, it has room to go up.

The `speed_control.h` header defines the API: `speed_init()`, `speed_set_target(int percent)`, `speed_get_current()`. The current speed is tracked as a float internally so that the Pi can query it for telemetry.

I also added a telemetry response feature. After each speed command, the ESP32 sends back a JSON response: `{"cmd": "speed_report", "current": 50, "target": 50, "pwm_duty": 127, "rpm": 0}`. The RPM field is placeholder for when we have encoders (v2.3). For now it's always 0.

## Why it changed

The hardcoded mapping in v2.0 was fine for testing, but it had problems:
- No way to calibrate the motor response curve. Our motor has significant nonlinearity—at low PWM, the motor doesn't spin at all until about 15% duty, then it jumps to about 200 RPM.
- No feedback to the Pi about what speed was actually set. The Pi would send `speed=50` and assume the motor was at 50%, but if the ESP32 clamped or ramped differently, the Pi had no way to know.
- No dead zone meant the motor sometimes hummed without moving, wasting battery and making an annoying buzz.

The formal speed control module fixes all three.

## Errors encountered

The error that bit me hardest was the PWM frequency. I configured the MCPWM timer for the motor at 50 Hz, same as the servo. The motor immediately emitted a loud, high-pitched whine—an audible 50 Hz hum from the stator windings. This is expected for low-frequency PWM: the motor windings act as an inductor, and at 50 Hz, the current ripple is large enough to produce audible mechanical vibration.

I tried increasing the frequency to 1000 Hz (1 kHz), which is still low enough for the motor driver (the L298N can handle up to 40 kHz) but well above human hearing. The whine disappeared completely. The motor ran silently.

But then the servo stopped working.

The servo requires a 50 Hz signal. A 1000 Hz signal confuses the servo controller—it sees pulses that are much shorter than the standard 1-2 ms it expects, and it either jitters or holds its last position. I had been using the same MCPWM unit for both motor and servo, and they share the same timer base.

I tried using two separate MCPWM timers with different frequencies. The ESP32's MCPWM peripheral has three timers. Timer 0 was configured for 50 Hz (servo), Timer 1 for the motor. But the MCPWM unit only has one clock divider. Both timers derive their frequency from the same 160 MHz APB clock with the same prescaler. To get 50 Hz on one timer and 1000 Hz on the other, you need different counter periods—which is possible with the MCPWM configuration. I set timer 0's period to 31250 (for 50 Hz at 1.5625 MHz resolution) and timer 1's period to 1562 (for 1 kHz at the same resolution). But the resolution (number of steps for duty cycle) depends on the period. At 50 Hz, you get 31250 steps (more than enough). At 1 kHz, you only get 1562 steps (about 10.5 bits). That's still fine for motor control—we don't need 12-bit resolution for a DC motor.

The error message that told me I was using the wrong configuration:

```
E (1234) MCPWM: MCPWM_OPR_A (timer 1) does not support the same prescale as timer 0
E (1235) MCPWM: requested duty value exceeds the timer's resolution
```

Wait, that second error isn't real—that's just me paraphrasing. The actual error from the ESP-IDF was:

```
E (1234) mcpwm: mcpwm_set_duty(317): timer is not configured
```

Turns out I hadn't called `mcpwm_init()` for timer 1. Once I initialized both timers separately with different frequencies, it worked.

But then I noticed a different problem: the motor at 1 kHz sounded fine, but the L298N was getting noticeably hotter. At 50 Hz, the inductive load keeps current flowing during the off-cycle. At 1 kHz, the switching losses in the H-bridge increase because the MOSFETs switch on and off more frequently. The L298N datasheet says switching losses become significant above 200 Hz. Running at 1 kHz would eventually overheat the driver.

I went back to 50 Hz for the motor. The audible whine is annoying but not harmful. The motor's rated for continuous operation at 50 Hz PWM. The servo needs 50 Hz. So everything stays at 50 Hz. I'll just wear earplugs.

## Alternative approaches considered

1. **Separate PWM generators**: Use one MCPWM unit for the servo and a separate PWM generator (e.g., the LEDC peripheral) for the motor. The ESP32's LEDC peripheral can generate PWM at any frequency independently of MCPWM. This would let me run the motor at 1 kHz and the servo at 50 Hz. I implemented this and it worked, but it uses extra hardware resources that I might need for other things (e.g., RGB LEDs for status indication).

2. **Higher-frequency servo**: Replace the standard servo with a digital servo that accepts higher PWM frequencies. Digital servos can operate at up to 333 Hz. But we only have analog servos from the kit.

3. **Active filtering**: Add an LC filter between the motor driver and the motor to smooth the PWM and eliminate the whine. This works but adds components.

I ultimately kept the motor at 50 Hz because the whine is a cosmetic issue, not a functional one. The robot will be driving on a competition floor with crowd noise anyway.

## Reasoning

The speed control module is the foundation for everything that follows. v2.3 (odometry) needs accurate speed reporting. v2.4 (PID) needs to adjust speed based on heading error. Having a clean API with dead zone, soft limits, and telemetry feedback makes all of that possible.

I also added a `speed_control_calibrate()` function that runs the motor at various PWM values and measures the actual RPM (using a tachometer). This generates a calibration curve that maps desired speed to actual PWM duty, compensating for the motor's nonlinear response. The calibration data is stored in NVS (Non-Volatile Storage) on the ESP32 so it persists across reboots.
