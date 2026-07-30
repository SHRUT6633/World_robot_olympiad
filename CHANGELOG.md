# Changelog — WRO 2026 4WS Robot

All notable changes to this project are documented below.

---

## [v1.4] — 2026-07-30 — Commented & Competition-Ready

### Added
- `pi/perception/pillar_detector.py` — Colour pillar detection from exact rulebook RGB: red(238,39,55), green(68,214,44), magenta(255,0,255)
- `pi/perception/pillar_tracker.py` — Pillar pass-side tracking with configurable NORMAL/REVERSED logic
- `pi/perception/parking_detector.py` — Full parking state machine: IDLE → MARKER_SEEN → BETWEEN_MARKERS → ALIGNING → BACKING_IN → PARKED → VERIFIED
- `pi/localization/track_map.py` — Lightweight geometry-based track section tracking (no SLAM)
- `config/surprise_rules.yaml` — Centralised Surprise Rule configuration (8 scenarios)
- `pi/dynamics/steering_modes.py` — Three 4WS steering modes: SAME_PHASE, OPPOSITE_PHASE, CRAB_WALK
- `docs/competition/` — Scoring criteria mapped to Appendix C (Mobility 4pts, Power/Sense 4pts, Obstacle 4pts)
- `docs/engineering/` — Failure analysis and incident reports

### Fixed
- Issue 003: I2C sensor error spam — rate-limited logging with auto-disable after 50 consecutive failures

### Changed
- `pi/main.py` — Wires pillar detection, parking detection, track map into the 50 Hz perception loop
- `pi/mission/state_machine.py` — Added parking sub-states (PARK_APPROACH, PARK_ALIGN, PARK_BACK_IN, PARK_VERIFY)
- `config/surprise_rules.yaml` — Added parking parameters (robot length, parallel tolerance, timeout) and track dimensions

---

## [v1.3] — 2026-07-29 — Fusion Bugfix & Filter Tuning

### Fixed
- **Critical**: `MerkedScaledSigmaPoints` → `MerweScaledSigmaPoints` typo in `pi/fusion/ukf.py:37` prevented UKF initialisation
- Issue 002: Infinite `while True` loops removed from all 7 scheduler callbacks — Ctrl+C now stops cleanly

### Added
- `docs/competition/mobility.md` — Proof of rule compliance scored at 4 pts (Appendix C.1)
- `docs/competition/power_sense.md` — Proof of rule compliance scored at 4 pts (Appendix C.2)
- `docs/competition/obstacle.md` — Proof of rule compliance scored at 4 pts (Appendix C.3)

---

## [v1.2] — 2026-07-29 — Import Restructuring

### Fixed
- `pi/main.py:14` — Missing `sys.path.insert(0, ...)` and missing `pi.` package prefix on imports prevented module loading

### Changed
- All imports in `pi/main.py` now use explicit `pi.` prefix
- `sys.path` setup added before any local imports

---

## [v1.1] — 2026-07-28 — L298N Driver & Self-Test

### Added
- `esp/main/l298n.c` — Single-motor L298N driver with PWM + direction control (Rule 11.3, 11.5 compliant)
- `esp/main/selftest.c` — Startup self-test: servo sweep + motor spin verification
- `esp/main/failsafe.c` — Safe stop on timeout or fault

### Changed
- `esp/main/uart_receiver.c` — Increased buffer, added CRC-16 validation
- `esp/main/packet_validator.c` — Range checking for all commands

---

## [v1.0] — 2026-07-27 — Initial Release

### Added
- Complete ESP32-S3 firmware (ESP-IDF v5.5.5): UART receiver, servo PWM, motor control, watchdog
- Complete Raspberry Pi software stack: sensor drivers (camera, ToF, IMU, magnetometer), UKF fusion, lane/wall detection, Stanley control, cubic-spline planning, UART communication
- Three config files (`pi_config.yaml`, `esp_config.yaml`, `surprise_rules.yaml`)
- Wiring documentation (`docs/wiring/`)
- Error reference catalog (`issues/005-error-reference-catalog.txt`)
