# The Full Journey — 90 Versions of Growth

```
v1.x          v2.x          v3.x          v4.x          v5.x
FOUNDATION    DRIVING       SENSING       TRACK         FUSION

v6.x          v7.x          v8.x          v9.x
CONTROL       MISSION       ADVANCED      POLISH
```

This folder holds 90 snapshots of our robot software as it grew from
nothing into something that drives, sees, thinks, and parks.

Each version folder has:
- `CHANGE.md` — what changed, why, what broke, how we fixed it
- Code files — the actual code at that moment (warts and all)

---

## The 9 Major Phases

### v1.x — Foundation & Hardware Testing
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 1.0 | Project skeleton — Pi + ESP32-S3 | Import path wrong, fixed with sys.path |
| 1.1 | I2C bus scanner — detect all sensors | IOError on missing sensor, try/except |
| 1.2 | Camera capture test | First frame black, 2s warmup |
| 1.3 | Motor spin test — L298N forward/reverse | ENA not PWM, motor only forward |
| 1.4 | Servo calibration sweep | Jitter at extremes, limit ±30° |
| 1.5 | UART ping-pong loopback | Lost first byte, flush buffer |
| 1.6 | Multi-sensor read loop | I2C contention, stagger reads 10ms |
| 1.7 | GPIO LED + switch debounce | Switch bounce, 50ms debounce |
| 1.8 | Startup self-test sequence | Camera slow, parallel threads |
| 1.9 | Hardware verification report | All 14 components tested PASS |

### v2.x — Basic Driving
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 2.0 | Forward drive command | Brownout at full PWM, ramp up 500ms |
| 2.1 | Turn + steering test | Ackermann error, inside/outside angles |
| 2.2 | PWM speed control | Audible whine at 50Hz, kept for servo |
| 2.3 | Wheel encoder odometry | Missed interrupts, hardware counter |
| 2.4 | PID straight line | Integral windup, anti-windup clamp |
| 2.5 | Open-loop trajectory | Timing drift, elapsed-time scheduling |
| 2.6 | Stop and reverse | Coast 30cm, dynamic braking |
| 2.7 | Speed ramping S-curve | Wheel slip, sinusoidal acceleration |
| 2.8 | Keyboard remote control | Key repeat jerky, poll state |
| 2.9 | Drive reliability summary | Max speed 1.8m/s, min radius 0.5m |

### v3.x — Sensing The World
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 3.0 | IMU raw data logging | First readings garbage, discard 100 |
| 3.1 | IMU calibration (bias + scale) | Bias drifts with temp, auto-recal |
| 3.2 | Complementary filter | 1s lag alpha=0.98, alpha=0.92 |
| 3.3 | Magnetometer heading | Hard iron distortion, 360° calib |
| 3.4 | ToF distance reading | Returns 0 when out of range, clamp |
| 3.5 | Multi-ToF fusion | Crosstalk simultaneous, stagger 20ms |
| 3.6 | Camera frame capture | Stalls after 100 frames, release buffer |
| 3.7 | RGB→HSV colour detection | Red wraps hue, two-range mask |
| 3.8 | Blob detection pillars | Floor reflections, aspect ratio filter |
| 3.9 | Sensor health monitor | Log spam, rate-limit 1 per 2s |

### v4.x — Understanding The Track
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 4.0 | Lane detection (Hough) | Noisy lines, average 5 frames |
| 4.1 | Wall detection from ToF | Blind spot <30mm, report 0mm |
| 4.2 | Free space detection | Shadows = obstacles, use saturation |
| 4.3 | Corner detection (gyro) | Drift 85-95°, reset after each corner |
| 4.4 | Red pillar detection Rule 13.21 | Red tape false positives, aspect ratio |
| 4.5 | Green pillar detection Rule 13.22 | Merges with floor, tune HSV venue |
| 4.6 | Pink marker detection Rule 13.27 | Too small far away, detect <500mm |
| 4.7 | Pillar distance from pixel height | Camera angle error, IMU pitch correct |
| 4.8 | Multi-pillar tracking | Disappear during turns, Kalman predict |
| 4.9 | Visual odometry | Too slow 5fps, 320×240 FAST corners |

### v5.x — Localization & Fusion
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 5.0 | Dead reckoning from encoders | Quadratic error 5cm→20cm, short only |
| 5.1 | Mag heading + gyro fusion | Motor interferes, disable while driving |
| 5.2 | Full complementary filter | Diverges >90°/s, reduce gyro trust |
| 5.3 | EKF implementation | Linearization error in turns |
| 5.4 | UKF implementation | Typo Merked→Merwe, one-letter bug |
| 5.5 | UKF tuning Q/R | Oscillating estimate, Q=1e-3 R=1e-1 |
| 5.6 | Adaptive noise estimation | Wild oscillation, EMA alpha=0.1 |
| 5.7 | Mahalanobis outlier rejection | Rejects 30% good, chi2 95% threshold |
| 5.8 | Cross-sensor verification | 5cm camera offset, calibrate transform |
| 5.9 | Pose pipeline integration | Too slow 20Hz, predict 100Hz correct 50Hz |

### v6.x — Control & Planning
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 6.0 | PID speed control | Oscillation low speed, gain schedule |
| 6.1 | PID servo position | Overshoot 5°, add D-term |
| 6.2 | Stanley steering control | Oscillate low speed, speed gain |
| 6.3 | Feedforward steering | Overshoot corners, limit 50% |
| 6.4 | Gain scheduling | Abrupt jerk, linear interpolation |
| 6.5 | Anti-windup PID | Slow response, conditional integration |
| 6.6 | Global planner waypoints | Cut corners, 100mm interpolation |
| 6.7 | Cubic spline trajectory | Runge overshoot, clamped boundaries |
| 6.8 | Velocity profiling | Wheel slip, 0.5m/s² limit |
| 6.9 | Obstacle avoidance | 200ms replan lag, precompute 3 paths |

### v7.x — Mission & Behavior
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 7.0 | Basic 4-state machine | if/elif chain, refactor to dict |
| 7.1 | Full 10-state machine | Timer overflow, reset on transition |
| 7.2 | Lap counter | Double count, 50cm hysteresis |
| 7.3 | Start/finish detection | Switch bounce, hardware+software |
| 7.4 | Obstacle pass strategy | Changes mid-avoid, lock until passed |
| 7.5 | Direction detection CW/CCW | Can't tell on straight, wait for corner |
| 7.6 | Reverse logic | Backs into wall, limit 20cm |
| 7.7 | Parking state machine | Misaligned, average 3 ToF readings |
| 7.8 | Race strategy | Too conservative, boost after lap 1 |
| 7.9 | Checkpoint manager | Late detection, look-ahead distance |

### v8.x — Advanced Features
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 8.0 | Same-phase steering | Wheel scrub, limit 25° |
| 8.1 | Opposite-phase steering | Confuses controller, slow to 0.3m/s |
| 8.2 | Crab-walk steering | IMU drift, disable yaw correction |
| 8.3 | Surprise rule YAML config | UTF-8 encoding, force utf-8 |
| 8.4 | Pillar pass-side tracker | Double count, 500ms cooldown |
| 8.5 | Full parking detector | Shadow fails, exposure compensation |
| 8.6 | Track map geometry | Distance error, reset at start line |
| 8.7 | Multi-rate scheduler | Async drift, absolute time scheduling |
| 8.8 | Health monitor heartbeats | False positives, 3 misses tolerance |
| 8.9 | Rate-limited error logger | Drops important errors, severity levels |

### v9.x — Polish & Competition Ready
| Ver | What We Did | Key Error Fixed |
|-----|-------------|-----------------|
| 9.0 | Full code comments | Stale comments, wrote after stable |
| 9.1 | Competition scoring docs | Claims without evidence, file:line refs |
| 9.2 | Error reference catalog | Can't reproduce, code analysis |
| 9.3 | README + ARCHITECTURE.md | Diagrams ugly, 80-char ASCII |
| 9.4 | CI pipeline GitHub Actions | Windows failure, ubuntu-latest |
| 9.5 | Repository cleanup | Nested artifacts, verify porcelain |
| 9.6 | Integration test | 5min slow, @pytest.mark.slow |
| 9.7 | Bug fixes (12 bugs) | Off-by-one, div-by-zero, null ptr |
| 9.8 | Performance optimization | 40% CPU reduction, configurable rates |
| 9.9 | Release candidate | 3 final bugs, all fixed |

---

## What Each Folder Contains

Every version folder has at least:
- `CHANGE.md` — the story: what, why, error, fix, alternatives, lesson
- Code files showing how the robot looked at that exact moment

The code is NOT always correct. That is the point. You see bugs as they
happened, fixes as we applied them, and the robot growing version by version.

---

## By The Numbers

| Metric | Value |
|--------|-------|
| Total versions | 90 (v1.0 → v9.9) |
| Development phases | 9 major phases |
| Bugs documented | 85+ (one per version, some have more) |
| Code files | 250+ |
| Total words in CHANGE.md files | 90,000+ |
| Hardware components verified | 14 |
| Maximum speed achieved | 1.8 m/s |
| Minimum turning radius | 0.5m (opposite-phase) |
| UKF state dimensions | 6 (x, y, heading, speed, accel, yaw_rate) |
| Steering modes | 3 (same-phase, opposite-phase, crab-walk) |
| Parking precision | ±2cm parallel tolerance |
| Configuration options | 55+ |
| Competition max score target | 122/122 pts |
