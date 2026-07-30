# WRO 2026 Future Engineers — 4WS AWD Self-Driving Robot

Team repository for the World Robot Olympiad 2026 Future Engineers category.
Autonomous 4-wheel steering (4WS), all-wheel drive (AWD) vehicle using
Raspberry Pi + ESP32-S3 with computer vision, sensor fusion, and PID control.

## Repository Structure

```
config/                  # YAML configuration files
  pi_config.yaml         #   Main Pi config (sensors, control, comms)
  surprise_rules.yaml    #   Surprise Rule adaptation (change 1 line)
docs/
  competition/           # Scoring-area documentation
    01_mobility.md       #   Mobility Management (4 pts)
    02_power_sense.md    #   Power & Sense Management (4 pts)
    03_obstacle.md       #   Obstacle Management (4 pts)
esp/                     # ESP32-S3 firmware (ESP-IDF)
  main/
    main.c               #   Entry point, state machine, packet protocol
    l298n.c/h            #   Single-motor L298N driver (Rule 11.3/11.5)
    servo.c/h            #   Single-servo steering (4WS via linkage)
pi/                      # Raspberry Pi software (Python)
  main.py                #   Entry point — async scheduler
  boot.py                #   Power-on self test
  sensors/               #   All sensor drivers
    camera/              #     PiCamera driver + calibration + pipeline
    tof/                 #     VL53L0X (short) + VL53L1X (long) ToF
    imu/                 #     MPU6050 (6-DoF IMU)
    magnetometer/        #     QMC5883L (magnetometer)
    base.py              #     SensorBase + rate-limited error logging
  perception/
    pillar_detector.py   #     Colour pillar detection (RGB from rules)
    lane_detection.py    #     Lane boundary detection
    wall_detection.py    #     ToF-based wall detection
    free_space.py        #     Drivable area classification
  fusion/                #   Sensor fusion
    ukf.py               #     Unscented Kalman Filter (6-DoF state)
    complementary.py     #     Complementary filter (pitch/roll/yaw)
  localization/          #   State estimation
    robot_localization.py #     High-level pose manager
  control/               #   Control algorithms
    stanley.py           #     Stanley lateral controller
    servo_pid.py         #     Servo position PID
    motor_pid.py         #     Motor speed PID
    adaptive_pid.py      #     Adaptive PID base class
  dynamics/              #   Vehicle dynamics
    kinematic_model.py   #     4WS kinematic model with steering modes
    steering_modes.py    #     Same-phase, opposite-phase, crab-walk
  mission/               #   Behaviour logic
    state_machine.py     #     Finite state machine (8+ states)
    lap_counter.py       #     Lap counting
  planning/              #   Path planning
    global_planner.py    #     Waypoint generation
  comm/                  #   Communication
    uart.py              #     UART to ESP32 (/dev/serial0 @ 115200)
    protocol.py          #     Packet protocol (CRC-16, header/footer)
  system/                #   System services
    manager.py           #     SystemManager (orchestrator)
    logger.py            #     Singleton logger (console + file)
    scheduler.py         #     Async task scheduler
    config_manager.py    #     YAML config loader
    health_monitor.py    #     Heartbeat-based health tracking
issues/                  # Error catalog (when/why/how each error occurs)
v1.x/                    # Version snapshots (v1.0 to v1.4)
```

## WRO Rule Compliance

| Rule | Requirement | Our Solution |
|------|-------------|-------------|
| 11.1 | Size ≤ 300×200×300 mm | Custom chassis within limits |
| 11.2 | Weight ≤ 1.5 kg | 3D-printed frame, lightweight |
| 11.3 | One steering actuator | Single servo + 4WS mechanical linkage |
| 11.4 | No omni wheels | Standard rubber wheels |
| 11.5 | No electronic differential | Single motor drives all 4 wheels (mechanical AWD) |
| 11.6 | Autonomous only | No RF/BT/WiFi during rounds |
| 11.10 | Wireless off | All radios disabled in code |
| 11.13 | Max 2 driving motors, mechanically linked | 1 motor for all 4 wheels |

## Surprise Rule Adaptation

Edit `config/surprise_rules.yaml` — ONE LINE changes adapt to any announced
surprise rule. See `docs/competition/03_obstacle.md` for scenarios.

Quick reference:
- Pillar colour swap   → `pillar_logic: "REVERSED"`
- Force drive direction → `drive_direction: "CCW"`
- Narrow track (600 mm) → `narrow_track: "ENABLED"`
- Tighter turns        → `steering_mode: "OPPOSITE_PHASE"`
- Parking changes      → `parking_mode: "REVERSED"`
- Speed limit          → `max_speed_ms: 1.0`

## Build & Run

### Raspberry Pi
```bash
cd ~/wro
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
python pi/main.py          # Race mode
python pi/boot.py          # With self-test + start switch
```

### ESP32 (ESP-IDF 5.5.5)
```bash
cd esp
. $HOME/esp/esp-idf/export.sh
idf.py build flash monitor
```

### Calibration
- Colour thresholds: edit `config/surprise_rules.yaml` → `colour_thresholds`
- Servo centering: run `python pi/tools/servo_calibrate.py`
- IMU bias: auto-calibrated at startup in `mpu6050.py`

## Scoring (Rule 10 + Appendix C)

The rubric evaluates 8 areas (30 pts documentation + 92 pts on-track):

| Area | Max | Our Approach |
|------|-----|-------------|
| Mobility Mgmt | 4 | 4WS with 3 steering modes, single servo, single motor |
| Power & Sense | 4 | 6 sensors, UKF fusion, battery isolation, wiring diagram |
| Obstacle Mgmt | 4 | State machine, colour pillar detection, ToF walls |
| Team Photos | 4 | All sides + team photo |
| Videos | 4 | 30s+ autonomous driving demos |
| GitHub Usage | 4 | Structured repo, frequent commits, public |
| Engineering Factor | 4 | Custom 3D-printed chassis, own PCB |
| Judge Impression | 2 | Clear docs, easy to duplicate |
| On-track Open | 30 | 3 laps in 3 min |
| On-track Obstacle | 62 | Pillars + parking in 3 min |
| **Total** | **122** | |

## Error Reference

Every possible error is documented in `issues/` with:
- Exact error message
- When it occurs
- Root cause
- Which file/line
- How the code handles it
- Severity (LOW / MEDIUM / HIGH / CRITICAL)

## Sensor Fusion Pipeline

```
Camera ─→ PillarDetector ─→ (pillar colour, bearing, distance)
       ─→ LaneDetector   ─→ (lane boundaries)
ToF    ─→ WallDetector   ─→ (wall proximity flags)
IMU    ─→ Complementary  ─→ (pitch, roll)
       ─→ UKF            ─→ (x, y, heading, v, yaw_rate)
Mag    ─→ Heading        ─→ (absolute yaw reference)
                              ↓
                        RobotLocalization.pose
```

## Dependencies

### Pi (Python)
- opencv-python (4.9+), numpy, smbus2, pyserial, pyyaml, filterpy

### ESP32 (ESP-IDF)
- ESP-IDF v5.5.5, LEDC (PWM), UART, GPIO, I2C (if expanded)

## Engineering Journal

See `docs/competition/` for detailed reasoning behind every design decision.
The journal maps directly to Appendix C scoring criteria.
