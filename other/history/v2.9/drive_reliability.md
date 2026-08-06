# Drive Reliability — v2.9 Summary

## Architecture

```
Pi 5 (high-level control)
  |-- UART 115200 baud
  v
ESP32-S3 (low-level control)
  |-- MCPWM Timer 0 (50 Hz): Servo steering (GPIO 13)
  |-- MCPWM Timer 1 (50 Hz): Drive motor (GPIO 25)
  |-- PCNT Unit 0: Left encoder (GPIO 4)
  |-- PCNT Unit 1: Right encoder (GPIO 5)
  |-- I2C: BMI270 IMU (0x68), AS5600 encoders (0x36)
```

## Performance Benchmarks

### Maximum Speed
- Top speed: **1.8 m/s** at 100% PWM
- Measured over 10 m straight run
- Confirmed with odometry and manual stopwatch
- S-curve ramp time: 500 ms to reach full speed

### Minimum Turning Radius
| Steering mode | Radius | Technique |
|---|---|---|
| SAME_PHASE | 0.8 m | Both front wheels steer same direction, differential drive for fine adjustment |
| OPPOSITE_PHASE | 0.5 m | One motor forward, one reverse; hard on motors but tightest turn |

### Odometry Accuracy
| Condition | Error over 10 m | Error over path with turns |
|---|---|---|
| Straight, smooth floor | ±2% | ±3% |
| Straight, rough floor | ±3% | ±5% |
| With S-curve accel | ±2% | ±4% |

Source of error: wheel slip during acceleration and turning.

### PID Heading Hold
| Speed | Heading error (RMS) |
|---|---|
| 0.5 m/s | ±1.2° |
| 1.0 m/s | ±3.5° |
| 1.5 m/s | ±5.1° |

Higher error at speed is due to tire slip during aggressive corrections.

### Stop Distance (with dynamic brake)
| Speed | Normal stop (coast) | Soft brake (50 ms) | Emergency (200 ms) |
|---|---|---|---|
| 10% | 3 cm | 2 cm | 1 cm |
| 25% | 8 cm | 4 cm | 2 cm |
| 50% | 16 cm | 7 cm | 4 cm |
| 75% | 24 cm | 10 cm | 7 cm |
| 100% | 32 cm | 12 cm | 9 cm |

### Brownout Test
- 50 consecutive start-stop cycles from 0 to 100% speed
- Zero brownouts detected
- Voltage measured at ESP32 3.3V rail: min 3.28V during ramp

### UART Reliability
- 50 Hz continuous operation for 10 minutes
- 30,000 messages sent
- 0 messages lost (after buffer increase to 1024 bytes)

## Known Issues

1. **PWM audible whine**: Motor PWM at 50 Hz produces audible 50 Hz hum. Cannot change because servo requires 50 Hz and L298N overheats above 200 Hz. Cosmetic only.

2. **L298N thermal limit**: More than 5 dynamic brake stops in quick succession (< 5 seconds apart) triggers thermal shutdown. 1-second cooldown between brakes prevents this. Emergency stops bypass cooldown.

3. **IMU heading drift**: ~0.1 degree per minute after bias calibration. Acceptable for course runs under 2 minutes. Longer runs require visual correction from camera.

4. **Servo nonlinearity**: ±2 degree error at extreme steering angles. Calibration lookup table compensates. Residual error is within tolerance.

5. **Odometry slip**: 2-5% error from wheel slip on smooth surfaces. Camera-based correction in next phase will compensate.

## All Bugs Fixed

| Bug | Version | Fix |
|---|---|---|
| ESP32 brownout at full PWM | v2.0 | Speed ramp-up over 500ms |
| Turning radius too large | v2.1 | Proper inside/outside Ackermann angles |
| PWM whine | v2.2 | Accepted at 50 Hz (servo requirement) |
| Missed encoder interrupts | v2.3 | Hardware PCNT counter |
| Integral windup in PID | v2.4 | Clamp + conditional integration |
| Timing drift in trajectory | v2.5 | Elapsed time instead of counter |
| Robot coasts 30cm before stop | v2.6 | Dynamic braking (reverse polarity) |
| Wheel slip at ramp start | v2.7 | S-curve acceleration (smoothstep) |
| Jerky keyboard movement | v2.8 | Poll key state instead of events |
| UART buffer overflow | v2.9 | Increased buffer to 1024 bytes |
| Gyro drift causing PID offset | v2.9 | Startup bias calibration |

## Test Log (Excerpt)

```
[2026-07-28 14:32] Speed test: 10m straight, 100% PWM. Time: 5.56s. Avg speed: 1.80 m/s
[2026-07-28 14:35] Turn test (SAME_PHASE): 30 deg steer, radius 0.82 m (target 0.80 m)
[2026-07-28 14:38] Turn test (OPPOSITE_PHASE): radius 0.51 m (target 0.50 m)
[2026-07-28 14:42] Stop test: 100% speed, normal stop -> 31 cm, brake stop -> 12 cm
[2026-07-28 14:45] Brownout test: 50 cycles, 0 brownouts
[2026-07-28 14:50] UART stress test: 30000 msgs, 0 lost
[2026-07-28 14:55] PID test: 1 m/s, heading RMS 3.2 deg over 2 min
[2026-07-28 15:00] Odometry test: 10m straight, error 1.8%
```

## Next Phase

v3.0 will begin Basic Line Following. The vision pipeline will run on the Pi 5, detecting a black line on a white surface from the camera feed, computing a cross-track error, and sending steering corrections to the drive system developed in v2.x.

The API contract for v3.x:
- `drive(speed, steering_angle)` — set both simultaneously
- `stop(brake=True)` — stop with optional dynamic brake
- `get_odometry()` — returns distance, heading, tick counts
- `get_heading()` — returns IMU heading
- `set_speed_ramp(ramp_time_ms)` — configure acceleration profile
