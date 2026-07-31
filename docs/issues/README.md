# WRO 4WS Robot — Known Issues

The complete engineering journal of every bug, error, and unexpected
behaviour encountered during the 90-version development of the WRO 2026
4WS robot (v1.0 → v9.9), from the first boot to the final race pipeline.

> **Main error catalog: [`000-error-catalog.txt`](000-error-catalog.txt)** —
> 1080 errors, one per development morning, deep detail. It is the first
> file in this folder for a reason: everything else supports it.

---

## Why this folder exists

Every error in this repo was **real**, was **seen in a terminal**, and
took **measured days** to fix. This folder exists so that:

1. Any error that appears on competition day has already been seen and
   documented — with the root cause and the fix.
2. The development history (90 versions) is verifiable against the
   failures that actually happened during it.
3. A new teammate can learn *why* the robot behaves the way it does,
   not just what to do.

```mermaid
flowchart LR
    A[90 versions<br/>v1.0 - v9.9] --> B[Development mornings<br/>180 days]
    B --> C[1080 errors logged]
    C --> D{SMALL or BIG?}
    D -->|SMALL<br/>1 day| E[Fixed same day]
    D -->|BIG<br/>2-5 days| F[Investigation steps<br/>before the fix]
    E --> G[000-error-catalog.txt]
    F --> G
```

---

## Folder structure

| Rank | File | Contents |
|------|------|----------|
| 1 | [`000-error-catalog.txt`](000-error-catalog.txt) | **Main catalog — 1080 errors**, deep detail, one per development morning |
| 2 | [`phases/`](phases/) | The same 1080 errors split into 9 phase files (120 each) for easy reading |
| 3 | `001-logger-init.txt` | `Logger.info()` fails because `.init()` never called |
| 4 | `002-ctrl-c-not-stopping.txt` | Ctrl+C does nothing because callbacks block the scheduler |
| 5 | `003-i2c-sensor-spam.txt` | I2C sensor errors flood the console at 600 lines/sec |
| 6 | `004-surprise-rule-flexibility.txt` | Surprise Rule config-driven adaptation |
| 7 | `005-error-reference-catalog.txt` | Classic error reference: every error, when/why/how |
| 8 | `README.md` | This index |

```mermaid
graph TD
    subgraph issues[docs/issues]
        A[000-error-catalog.txt]
        B[phases/]
        C[001-005 issue docs]
        D[README.md]
    end
    E[history/ v1.0-v9.9] -->|source of truth| A
    A -->|split by phase| B
    A -->|quick reference| C
    D -->|index + diagrams| A
```

---

## Anatomy of one error entry

Every entry in the catalog follows the same structure, so any error can
be read in 30 seconds:

```text
----------------------------------------------------------------------
E0009 | v9.0 | BIG | Found Day 162 | Fixed in 2 days | Pi SSH terminal
----------------------------------------------------------------------
File      : history/v9.0/esp_main.c
Error     : Full pipeline crashes 2 seconds after the start
Terminal  : Pi SSH terminal
  $ python3 main.py
  TypeError: 'Task' object is not callable        <- exact output
WHAT HAPPENED
  <the incident as it happened>
WHY IT HAPPENED (root cause)
  <the deep technical reason - the "why factor">
INVESTIGATION (before the fix)                    <- BIG errors only
  - <step 1 tried>
  - <step 2 tried>
  - <step 3 that found it>
FIX (took 2 days)
  <the solution>
```

| Field | Meaning |
|-------|---------|
| `E0001 … E1080` | Unique error ID, sequential |
| `vX.Y` | The version phase where it happened |
| `SMALL / BIG` | SMALL = 1 line of code, fixed same day; BIG = 2–5 days |
| `Found Day N` | Development-day counter (180 days total, **no dates**) |
| `Fixed in X day(s)` | How many days it took to fix |
| `Terminal` | Where it showed: Pi SSH, ESP32 serial monitor, CMD, etc. |
| `WHAT HAPPENED` | The incident |
| `WHY IT HAPPENED` | Root cause — the deep research |
| `INVESTIGATION` | The steps tried before the fix (BIG only) |
| `FIX` | The solution |

---

## The error lifecycle

How an error travelled from "morning surprise" to a catalog entry:

```mermaid
flowchart TD
    S[Morning: robot test starts] --> A[Error appears in terminal]
    A --> B{Recognized?}
    B -->|Yes - catalog match| C[Find the entry E-number]
    B -->|No - new error| D[Read the output]
    D --> E{How bad?}
    E -->|One line of code| F[SMALL - fix same day]
    E -->|Deep debugging| G[BIG - investigate]
    G --> H[Collect 3 investigation steps]
    H --> I[Find root cause - WHY]
    F --> J[Fix + verify on robot]
    I --> J
    J --> K[Log into template data]
    K --> L[Regenerate catalog]
    C --> M[Apply the documented fix]
    L --> M
    M --> N[Competition day: debug in minutes]
```

---

## Competition-day quick decision guide

The catalog's real purpose. When something fails at the venue:

```mermaid
flowchart TD
    Q[Something broke] --> R{What type?}
    R -->|I2C / sensor| S[See 005 reference<br/>+ search catalog for the sensor]
    R -->|Pi / SSH / boot| T[Check v1 phase: boot & SSH<br/>or v9: final pipeline]
    R -->|Motion / steering| U[Check v2 drive or v6 control phase]
    R -->|Vision / pillars| V[Check v4 perception phase]
    R -->|Position / heading| W[Check v5 localization phase]
    S --> X{Found entry?}
    T --> X
    U --> X
    V --> X
    W --> X
    X -->|Yes| Y[Read WHAT + WHY + FIX -<br/>apply the documented fix]
    X -->|No - never seen| Z[Log it as a new entry<br/>after the round]
```

---

## Phases overview

| Phase | Catalog file | Theme | Typical errors |
|-------|--------------|-------|----------------|
| v1.x | `phases/v1-boot-and-ssh.txt` | Boot & basics + SSH | Import failures, SSH refused, kernel panic, SD full |
| v2.x | `phases/v2-drive-and-motor.txt` | Drive & motor control | PWM overflow, brownout, encoder dead, odometry drift |
| v3.x | `phases/v3-imu-sensors.txt` | IMU / sensors | WHO_AM_I mismatch, ToF frozen, mag saturation |
| v4.x | `phases/v4-perception.txt` | Perception & vision | False pillars, HSV drift, lane flip, VO scale |
| v5.x | `phases/v5-localization.txt` | Localization & fusion | Singular matrix, NaN covariance, UKF divergence |
| v6.x | `phases/v6-control.txt` | Control & planning | Windup, oscillation, infeasible MPC |
| v7.x | `phases/v7-mission.txt` | Mission / state machine | Stuck states, double lap counts, early starts |
| v8.x | `phases/v8-integration.txt` | Integration & system | Scheduler deadlock, OOM, heartbeat failsafe |
| v9.x | `phases/v9-final-pipeline.txt` | Final pipeline (big) | Pipeline crash, watchdog loop, SD corruption |

```mermaid
timeline
    title Development days (180 total, counter only)
    v1.x : Day 1-20 : Boot, basics, first SSH fights
    v2.x : Day 21-40 : Motors, Ackermann, first drive
    v3.x : Day 41-60 : IMU, ToF, sensor fusion starts
    v4.x : Day 61-80 : Vision, pillars, corners
    v5.x : Day 81-100 : EKF/UKF, localization
    v6.x : Day 101-120 : Control loops, planning
    v7.x : Day 121-140 : State machine, mission logic
    v8.x : Day 141-160 : Integration, health, scheduling
    v9.x : Day 161-180 : Final pipeline, race readiness
```

---

## Statistics

| Metric | Value |
|--------|-------|
| Total errors | **1080** (720 SMALL / 360 BIG) |
| Development days | **180** (day counter, no dates) |
| Versions covered | **90** (v1.0 → v9.9) |
| Phases | **9** (v1.x → v9.x) |
| Errors per version | 12 (8 SMALL + 4 BIG) |
| Big errors with investigation | 360 (all) |

The distribution shows the pattern of the whole project: most bugs were
small and fixed the same day; the 360 big ones — the ones that cost
2–5 days each — are the ones worth studying before the competition.

---

## Regenerating the catalog

The catalog is **generated, not hand-written**, so it stays consistent.
The template data holds every error's WHAT/WHY/INVESTIGATION/FIX.

```
python scripts/generate_error_catalog.py
```

| File | Role |
|------|------|
| `scripts/error_catalog_data.py` | Template data — themes 1–5 (v1–v5) |
| `scripts/error_catalog_data2.py` | Template data — themes 6–9 (v6–v9) |
| `scripts/generate_error_catalog.py` | Generator — rendering, day counters, files |

The generator writes `000-error-catalog.txt` (combined) and all 9 files
under `phases/` in one pass. Adding a new error = add one template
tuple, regenerate, done.

## Documented top-10 hardest bugs

The BIG errors that cost the most days — read these before race day:

| Error | Found | Fixed | Root cause in one line |
|-------|-------|-------|------------------------|
| Kernel panic VFS mount fail | v1.x | 4 days | SD corruption after power cut |
| Brownout reboot loop | v2.x | 4 days | Battery sag below 2.8V at motor start |
| UKF diverges off track | v5.x | 5 days | Unbounded adaptive noise growth |
| Scheduler deadlock on exception | v8.x | 4 days | No exception boundary in the loop |
| OOM killer kills perception | v8.x | 3 days | Unbounded frame queue |
| ESP32 watchdog reset at 89s | v9.x | 3 days | Blocking UART write starved watchdog |
| SD corrupt on race day | v9.x | 5 days | ext4 journal lost on power cut |
| UART echo storm between boards | v9.x | 2 days | RX/TX swapped + no checksum |
| False pillar on scoreboard | v9.x | 3 days | Scoreboard passes every color gate |
| Camera frozen 40s on race morning | v9.x | 3 days | Blocking read, no timeout |
