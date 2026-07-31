# WRO 4WS Robot — Known Issues

Documentation of every bug, error, and unexpected behaviour encountered
during the 90-version development of the WRO 2026 robot (v1.0 → v9.9).
Use this folder to debug on competition day and to understand why each
failure happened.

## Folder structure

| File | Contents |
|------|----------|
| `README.md` | This index. Start here. |
| `001-logger-init.txt` | `Logger.info()` fails because `.init()` never called |
| `002-ctrl-c-not-stopping.txt` | Ctrl+C does nothing because callbacks block the scheduler |
| `003-i2c-sensor-spam.txt` | I2C sensor errors flood the console at 600 lines/sec |
| `004-surprise-rule-flexibility.txt` | Surprise Rule config-driven adaptation |
| `005-error-reference-catalog.txt` | Classic error reference: every error, when/why/how |
| `006-error-catalog-1000-plus.txt` | **1080 errors**, deep detail, one per development morning |
| `phases/` | The same 1080 errors split into 9 phase files for easy reading |

## The 1000+ error catalog

`006-error-catalog-1000-plus.txt` covers all 90 versions of the history
repo. Each entry records the day it appeared (a counter, no dates), the
terminal it showed in, the exact output, and a full analysis:

```
E0009 | v9.0 | BIG | Found Day 162 | Fixed in 2 days | Pi SSH terminal
File      : history/v9.0/esp_main.c
Error     : Full pipeline crashes 2 seconds after the start
Terminal  : Pi SSH terminal
  $ python3 main.py
  TypeError: 'Task' object is not callable
WHAT HAPPENED
  ...
WHY IT HAPPENED (root cause)
  ...
INVESTIGATION (before the fix)
  - ...
FIX (took 2 days)
  ...
```

- **SMALL errors** — one line of code, fixed the same day (1 day).
- **BIG errors** — deep debugging, took 2–5 days, include the
  investigation steps before the fix.
- **Day** — development-day counter only (180 days total, no dates).
- Every entry has a **WHY** section: the root cause and the deeper
  reason the failure occurred, so it looks and reads like a real
  engineering journal.

### Phases

| Phase | Folder file | Theme |
|-------|-------------|-------|
| v1.x | `phases/v1-boot-and-ssh.txt` | Boot & basics + SSH errors (Pi bot) |
| v2.x | `phases/v2-drive-and-motor.txt` | Drive & motor control |
| v3.x | `phases/v3-imu-sensors.txt` | IMU / sensor subsystem |
| v4.x | `phases/v4-perception.txt` | Perception & vision |
| v5.x | `phases/v5-localization.txt` | Localization & fusion |
| v6.x | `phases/v6-control.txt` | Control loop & planning |
| v7.x | `phases/v7-mission.txt` | Mission / state machine |
| v8.x | `phases/v8-integration.txt` | Integration & system |
| v9.x | `phases/v9-final-pipeline.txt` | Final pipeline (big errors) |

## Regenerating

The catalog is generated, not hand-written. After editing the template
data, regenerate everything with:

```
python scripts/generate_error_catalog.py
```

Template data lives in:

- `scripts/error_catalog_data.py`  — themes 1–5
- `scripts/error_catalog_data2.py` — themes 6–9
- `scripts/generate_error_catalog.py` — generator (rendering, days, files)
