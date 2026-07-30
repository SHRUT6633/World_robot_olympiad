# WRO 2026 Future Engineers — 4WS AWD Self-Driving Robot

[![CI](https://github.com/SHRUT6633/World_robot_olympiad/actions/workflows/ci.yml/badge.svg)](https://github.com/SHRUT6633/World_robot_olympiad/actions/workflows/ci.yml)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.5-blue)](https://github.com/espressif/esp-idf)
[![Python](https://img.shields.io/badge/Python-3.11+-brightgreen)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](LICENSE)
[![WRO 2026](https://img.shields.io/badge/WRO-2026-orange)](https://worldrobotolympiad.org)

Team repository for WRO 2026 Future Engineers. Autonomous 4WS AWD vehicle
using Raspberry Pi 4 + ESP32-S3 with computer vision and sensor fusion.

**Target score: 122/122 pts** (92 on-track + 30 documentation)

## Architecture
```
RASPBERRY PI 4 (Python)
  SENSORS -> FUSION -> PERCEPTION -> LOCALIZATION -> MISSION
                                                    |
  CONTROL <- DYNAMICS <- TRAJECTORY <- PLANNING <--+
      |
   UART -> ESP32-S3 (C/ESP-IDF) -> L298N + Servo
```

## Quick Start
```bash
# Pi
pip install -r requirements.txt
python pi/main.py

# ESP32
cd esp && idf.py build flash monitor
```

## Scoring
| Area | Pts | Approach |
|------|-----|----------|
| Mobility | 4 | 4WS, 3 steering modes, single servo/motor |
| Power & Sense | 4 | 6 sensors, UKF fusion, battery isolation |
| Obstacle | 4 | Colour pillar detection, state machine |
| On-track | 92 | 3 laps + pillars + parking in 3 min |
| **Total** | **122** | |

## Surprise Rule Adaptation
Edit `config/surprise_rules.yaml` — change ONE line to adapt:
- `pillar_logic: "REVERSED"` -> swap pillar colours
- `steering_mode: "OPPOSITE_PHASE"` -> tighter turns
- `max_speed_ms: 1.0` -> speed limit

## Key Files
- `pi/main.py` — Race entry point
- `esp/main/main.c` — ESP32 firmware
- `docs/competition/` — Appendix C scoring docs
- `docs/issues/error_catalog.md` — Error reference
- `config/surprise_rules.yaml` — Surprise rule config
