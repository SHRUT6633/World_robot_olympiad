# v9.9 — Release Candidate

## What Changed

This is it. The final version before competition day. Everything up to v9.8 was building, documenting, and optimising. v9.9 is the release candidate — the version we'll use at the World Robot Olympiad 2026.

The main deliverable is `RELEASE_NOTES.md`, a comprehensive document that summarises everything a judge (or a team member on competition day) needs to know. It covers:

1. **Feature summary** — All 12 key capabilities, from 4WS with three steering modes to the config-based Surprise Rule adaptation.
2. **Known issues** — 5 known limitations that we chose not to fix (acceptable tradeoffs for competition).
3. **Competition strategy** — How we'll approach each of the two challenges (Open and Obstacle) with specific tactics.
4. **Configuration checklist** — What to check on competition morning (battery voltage, config file, sensor alignment, etc.).
5. **Quick debug guide** — The 3 most likely things to go wrong and their 30-second fixes.
6. **Scoring prediction** — Our expected score breakdown.

Beyond the release notes, I also did a final review of every source file — reading the entire codebase from top to bottom, looking for any remaining bugs or inconsistencies.

## Errors Found and Fixed During Final Review

**Error 1: Wrong default value for `steering_mode` in config.**
In `main.py`, the fallback for steering mode was:
```python
steering_mode_str = sr.get("steering_mode", "SAME_PHASE")
```
But in `config/surprise_rules.yaml`, the steering_mode key was `steering_mode: "SAME_PHASE"` — correct in the file, but the code used a fallback string `"SAME_PHASE"` while the enum mapping used `getattr(SteeringMode, steering_mode_str, SteeringMode.SAME_PHASE)`. This worked for the default, but if a user typed `"same_phase"` (lowercase) in the config, `getattr` would silently fall back to `SteeringMode.SAME_PHASE` instead of warning about the typo.

**Fix:** Added a validation warning:
```python
steering_mode_str = sr.get("steering_mode", "SAME_PHASE")
if steering_mode_str not in SteeringMode.__members__:
    log.warn(f"Unknown steering_mode '{steering_mode_str}', using SAME_PHASE")
    steering_mode_str = "SAME_PHASE"
steering_mode = SteeringMode[steering_mode_str]
```

**Error 2: Missing import in `main.py`.**
In the latest version, the import for `PillarTracker` was commented out during a refactor:
```python
# from pi.perception.pillar_tracker import PillarTracker  # accidentally removed
```

The line was simply missing. When I ran the integration test, Python raised `NameError: name 'PillarTracker' is not defined` at the point where `pillar_tracker = PillarTracker(...)` is called.

**Fix:** Added the import back:
```python
from pi.perception.pillar_tracker import PillarTracker
```

**Error 3: Typo in pin number for green LED in `esp/main/main.c`.**
The green LED was defined as `GPIO 2` in the code comments, but the C define had:
```c
#define LED_GREEN_GPIO 2    // Correct: GPIO2 is the onboard LED
```
This was actually correct — I was confused during review. The "typo" I thought I found was that `TEST_LED_GPIO` in `selftest.c` is also GPIO 2, so both `main.c` and `selftest.c` use GPIO 2 for the LED. This works because they're in the same firmware — the self-test configures GPIO 2, and the LED indicator task also uses GPIO 2. No conflict.

But I DID find a real typo in `selftest.c`:
```c
#define TEST_LED_GPIO    2

// In test_led():
// Actually misnamed — this is a LED test, not a UART test.
// The result field is named "uart_ok" from an earlier refactor.
result->uart_ok = true;    // Should be result->led_ok or similar
```

This is a naming issue, not a functional bug. The self-test result struct has a field called `uart_ok`, but the test actually exercises the LED, not the UART. I documented this in the commit but didn't rename the field (too many downstream consumers).

## Final State of the Repository

```
config/pi_config.yaml        — Main Pi config (sensors, control)
config/surprise_rules.yaml   — Surprise Rule adaptation (change 1 line)
docs/competition/01_mobility.md   — Appendix C doc #1
docs/competition/02_power_sense.md  — Appendix C doc #2
docs/competition/03_obstacle.md     — Appendix C doc #3
docs/issues/error_catalog.md        — Error reference
esp/main/                    — ESP32-S3 firmware
pi/main.py                   — Pi race entry point
pi/                          — All Python modules (sensors, fusion, etc.)
tests/                       — Unit and integration tests
.github/workflows/ci.yml     — CI pipeline
README.md                    — Quick reference
ARCHITECTURE.md              — Architecture + data flow
RELEASE_NOTES.md             — Competition release notes
```

## Competition Strategy

1. **Morning of competition:** Check battery voltage (7.4V LiPo > 7.0V), verify config file, run self-test.
2. **Open Challenge:** SAME_PHASE steering, 2.0 m/s target speed, standard pillar logic.
3. **Obstacle Challenge:** OPPOSITE_PHASE for tight parking, watch for surprise rule announcement at 08:30.
4. **If surprise rule:**
   - Open `config/surprise_rules.yaml`
   - Change one line
   - Save and re-run `python pi/main.py`
   - Done in under 60 seconds.

## Scoring Prediction
- Mobility Management: 4/4
- Power & Sense Management: 4/4
- Obstacle Management: 4/4
- Team Photos: 4/4
- Videos: 4/4
- GitHub Usage: 4/4
- Engineering Factor: 4/4
- Judge Impression: 2/2
- On-track Open: 28/30 (conservative: some speed loss in corners)
- On-track Obstacle: 58/62 (conservative: parking may lose 4 pts)
- **Total: 116/122** (optimistic: 122/122 if everything goes perfectly)
