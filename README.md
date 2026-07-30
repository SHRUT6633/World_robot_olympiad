# WRO 2026 Future Engineers — 4WS AWD Self-Driving Robot

[![CI](https://github.com/SHRUT6633/World_robot_olympiad/actions/workflows/ci.yml/badge.svg)](https://github.com/SHRUT6633/World_robot_olympiad/actions/workflows/ci.yml)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.5-blue)](https://github.com/espressif/esp-idf)
[![Python](https://img.shields.io/badge/Python-3.11+-brightgreen)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](LICENSE)
[![WRO 2026](https://img.shields.io/badge/WRO-2026-orange)](https://worldrobotolympiad.org)

Team repository for the World Robot Olympiad 2026 **Future Engineers** category.
Autonomous 4-wheel steering (4WS), all-wheel drive (AWD) vehicle using
Raspberry Pi 4 + ESP32-S3 with computer vision, sensor fusion, and PID control.

**Target score: 122/122 pts** (92 on-track + 30 documentation)

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI 4 (Python)                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  SENSORS → FUSION → PERCEPTION → LOCALIZATION → MISSION  │  │
│  │                                              ↓            │  │
│  │  CONTROL ← DYNAMICS ← TRAJECTORY ← PLANNING ←           │  │
│  │      ↓                                                   │  │
│  │   UART ──────────────────────────────────────────→ ESP32  │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
                             │ UART @115200
                             ▼
┌────────────────────────────────────────────────────────────────┐
│                     ESP32-S3 (C/ESP-IDF)                       │
│  UART → CommandValidator → Servo PWM (1) + L298N Motor (1)    │
│  Watchdog (3s) + Failsafe + Self-Test                          │
└────────────────────────────────────────────────────────────────┘
```

Full architecture diagram: [`ARCHITECTURE.md`](ARCHITECTURE.md)

---

## Key Engineering Highlights

- Single DC motor drives all 4 wheels via mechanical AWD (**Rule 11.3/11.5** compliant)
- Single servo controls 4WS via mechanical linkage (**Rule 11.3** compliant)
- 3 steering modes: **SAME_PHASE** (default), **OPPOSITE_PHASE** (tight turns), **CRAB_WALK** (sideways parking)
- **Surprise Rule** adaptation via ONE config line change — no code edits
- Pillar colours from **exact rulebook RGB**: red(238,39,55), green(68,214,44), magenta(255,0,255)
- **Parking verification**: 30 s stationary + parallel alignment ≤ 2 cm for full 15 pts
- 6-DoF **Unscented Kalman Filter** (UKF) with adaptive noise estimation
- **Rate-limited error logging** — auto-disables sensors after 50 consecutive failures
- All callbacks are **single-iteration** — scheduler owns timing via `hz`, clean Ctrl+C shutdown
- **Error catalog** documents every possible error with cause, handling, and severity

---

## Repository Structure

```
config/                  # YAML configuration files
  pi_config.yaml         #   Main Pi config (sensors, control, comms)
  surprise_rules.yaml    #   Surprise Rule adaptation (change 1 line)
docs/
  competition/           # Scoring-area documentation (Appendix C)
    01_mobility.md       #   Mobility Management (4 pts)
    02_power_sense.md    #   Power & Sense Management (4 pts)
    03_obstacle.md       #   Obstacle Management (4 pts)
  engineering/
    FAILURE_ANALYSIS.md  #   All bugs, root causes, fixes, prevention
esp/                     # ESP32-S3 firmware (ESP-IDF v5.5.5)
  main/
    main.c               #   Entry point, state machine, packet protocol
    l298n.c/h            #   Single-motor L298N driver
    servo_pwm.c/h        #   Single-servo 50 Hz PWM
    uart_receiver.c/h    #   UART packet receiver + CRC validation
    failsafe.c/h         #   Safe stop on timeout/fault
    selftest.c/h         #   Startup self-test
pi/                      # Raspberry Pi software (Python)
  main.py                #   Entry point — async scheduler
  boot.py                #   Power-on self test
  sensors/               #   All sensor drivers
  fusion/                #   UKF + complementary filter + adaptive noise
  perception/            #   Pillar, lane, wall, free-space detection
  localization/          #   Pose estimation + track map
  control/               #   Stanley + PID controllers
  dynamics/              #   4WS kinematic model + steering modes
  mission/               #   State machine + lap counter
  planning/              #   Global planner + waypoints
  trajectory/            #   Cubic splines + velocity profiling
  comm/                  #   UART packet protocol
  system/                #   Scheduler, logger, health monitor, config
  hardware/              #   Status LED, start switch
issues/                  # Error reference catalog
v1.x/                    # Version snapshots (v1.0 → v1.4)
.changelog.md            # Full version history
ARCHITECTURE.md          # System architecture + data flow
.github/workflows/ci.yml # CI pipeline (lint, build, validate)
```

---

## WRO Rule Compliance

| Rule | Requirement | Our Solution |
|------|-------------|-------------|
| 11.1 | Size ≤ 300×200×300 mm | Custom chassis within limits |
| 11.2 | Weight ≤ 1.5 kg | 3D-printed frame, lightweight |
| 11.3 | One steering actuator | Single servo + 4WS mechanical linkage |
| 11.4 | No omni wheels | Standard rubber wheels |
| 11.5 | No electronic differential | Single motor drives all 4 wheels |
| 11.6 | Autonomous only | No RF/BT/WiFi during rounds |
| 11.10 | Wireless off | All radios disabled in code |
| 11.13 | Max 2 driving motors, mechanically linked | 1 motor for all 4 wheels |
| 13.21 | Red pillar | Detected at RGB(238,39,55) |
| 13.22 | Green pillar | Detected at RGB(68,214,44) |
| 13.25 | Magenta parking markers | Detected at RGB(255,0,255) |

---

## Surprise Rule Adaptation

Edit `config/surprise_rules.yaml` — change ONE line to adapt:

| Scenario | Config Change | Effect |
|----------|--------------|--------|
| Pillar colours swapped | `pillar_logic: "REVERSED"` | Red passed on right, green on left |
| Forced CCW driving | `drive_direction: "CCW"` | Reverse direction |
| Stealth (no magenta) | `parking_mode: "REVERSED"` | Reverse into parking |
| Narrow track | `narrow_track: "ENABLED"` | Tight turning (opposite-phase steer) |
| Speed limit | `max_speed_ms: 1.0` | Reduced max speed |
| No stop-and-go | `stop_and_go: "DISABLED"` | No stopping at pillars |
| Obstacle pass side forced | `obstacle_pass_side: "LEFT"` | Always pass left |
| Steering mode | `steering_mode: "OPPOSITE_PHASE"` | Tighter turns |

---

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

---

## Scoring (Rule 10 + Appendix C)

| Area | Max | Our Approach |
|------|-----|-------------|
| Mobility Mgmt | 4 | 4WS with 3 steering modes, single servo, single motor |
| Power & Sense | 4 | 6 sensors, UKF fusion, battery isolation, wiring diagram |
| Obstacle Mgmt | 4 | State machine, colour pillar detection, ToF walls |
| Team Photos | 4 | All sides + team photo |
| Videos | 4 | 30s+ autonomous driving demos |
| GitHub Usage | 4 | Structured repo, frequent commits, CI pipeline |
| Engineering Factor | 4 | Custom 3D-printed chassis, own PCB |
| Judge Impression | 2 | Clear docs, easy to duplicate |
| On-track Open | 30 | 3 laps in 3 min |
| On-track Obstacle | 62 | Pillars + parking in 3 min |
| **Total** | **122** | |

---

## Failure Analysis

Every bug encountered during development is documented in
[`docs/engineering/FAILURE_ANALYSIS.md`](docs/engineering/FAILURE_ANALYSIS.md)
with exact error messages, root causes, fixes applied, and prevention strategies.

| ID | Issue | Severity | Fix |
|----|-------|----------|-----|
| 001 | Logger `AttributeError` at startup | CRITICAL | Added `log.init()` |
| 002 | Ctrl+C doesn't stop robot | MAJOR | Removed `while True` from callbacks |
| 003 | I2C error log spam (116 msg/s) | MINOR | Rate-limited logging |
| 004 | ImportError on Pi | CRITICAL | Added `sys.path` + `pi.` prefix |
| 005 | Filterpy typo `Merked→Merwe` | CRITICAL | Fixed class name |
| 006 | Build artifacts in repo (2.1 GB) | COSMETIC | Updated `.gitignore` + `git rm` |

---

## Error Reference

Every possible error is catalogued in [`issues/`](issues/) with:
- Exact error message
- When it occurs
- Root cause
- File/line number
- How the code handles it
- Severity (LOW / MEDIUM / HIGH / CRITICAL)

---

## Engineering Journal

See [`docs/competition/`](docs/competition/) for detailed reasoning behind
every design decision, mapped directly to Appendix C scoring criteria.
