# v9.1 — Competition Scoring Docs

## What Changed

With the code fully documented (v9.0), it was time to prove to the judges that our robot actually does what we claim. The WRO 2026 Future Engineers rubric awards **12 points** across three "Appendix C" documentation areas: Mobility Management (4 pts), Power & Sense Management (4 pts), and Obstacle Management (4 pts). Each area requires clear mapping from the written description to the actual code.

I created three documents under `docs/competition/`:
- `01_mobility.md` — Documents steering modes, 4WS mechanical linkage, single-servo compliance with Rule 11.3, AWD compliance with Rule 11.5, and turning radius calculations.
- `02_power_sense.md` — Documents the dual-battery system, I2C sensor bus, UKF fusion pipeline, and how each sensor connects to the pose estimate.
- `03_obstacle.md` — Documents pillar detection by colour (exact Rulebook RGB values), the pass-left/pass-right logic, state machine transitions, and the parking verification procedure.

Each document follows a strict template:
1. **Which Appendix C criteria it addresses** — Explicit mapping to scoring sub-bullets.
2. **Technical description** — How the subsystem works, with key parameters.
3. **Code mapping** — Every claim is backed by a file:line reference. For example: "The Stanley controller (pi/control/stanley.py:25) computes steering angle as heading_error + arctan(k * crosstrack / (k_soft + v))."
4. **Evidence** — What a judge can look at to verify the claim (code, config files, test output, photo).
5. **Surprise Rule adaptation** — How this subsystem can adapt via config change.

I spent about 8 hours on this. The hardest part was tracing every claim back to the exact line — I kept finding features I thought existed but were actually only partially implemented.

## Errors Encountered and Fixed

**Error 1: "The judge requires evidence, not just claims."**
My first draft of `01_mobility.md` said: "Our robot supports three steering modes: SAME_PHASE, OPPOSITE_PHASE, and CRAB_WALK." A perfectly reasonable statement, but the judge would ask: "Where in the code is this implemented? How do I verify it?"

The first draft had no file references. It was just marketing copy. The judge's scoring rubric explicitly states: "Documents claims with links to the actual code."

**Fix:** I went through every paragraph and added file:line references. The steering modes paragraph became:

```
## Steering Modes (config/surprise_rules.yaml)
| Mode | Front | Rear | Use Case |
|------|-------|------|----------|
| SAME_PHASE | +delta | +delta | High-speed straights (pi/dynamics/steering_modes.py:12-14) |
| OPPOSITE_PHASE | +delta | -delta | Tight turns (pi/dynamics/steering_modes.py:15-18) |
| CRAB_WALK | +delta | +delta | Sideways parking (pi/dynamics/steering_modes.py:19-22) |
```

This is tedious but I think it's what wins points. The judge can literally open each file and see the code.

**Error 2: "This feature isn't actually implemented yet."**
While writing `02_power_sense.md`, I claimed we had "battery voltage monitoring via ADC on the ESP32." Then I went to look for the code... and it didn't exist. We had discussed adding it, created a placeholder in the config file, but never wrote the driver.

This was awkward. The doc was supposed to describe what EXISTS, not what we PLANNED.

**Fix:** I removed the battery monitoring claim entirely and replaced it with a truthful description of the dual-battery isolation approach (separate batteries for Pi/ESP vs motors/servo). I also created a GitHub issue to add voltage monitoring post-competition.

**Error 3: "The code has changed since you wrote the doc."**
Between writing `01_mobility.md` and `03_obstacle.md`, I made a minor change to `pillar_detector.py` (adjusted the green HSV range). Now the doc I'd already written for `02_power_sense.md` referenced the old line numbers. This is the same "stale comments" problem from v9.0, but now affecting documentation instead of inline comments.

**Fix:** I did all three docs in one sitting, WITHOUT making any code changes in between. After all three docs were written, I verified every file:line reference by reading the actual code. I also added a note in the README saying "Docs are snapshots as of v9.1 — always cross-reference with actual code."

## Alternatives Considered

1. **Auto-generated docs from docstrings.** I could have used Sphinx to parse docstrings and generate the competition docs automatically. This would have saved time but produced generic output. The competition docs need to explicitly address the Appendix C criteria, which is a different structure from code docstrings.

2. **Video walkthrough instead of written docs.** Some teams create a video explaining their code. But the rubric explicitly mentions "written documentation that maps code to criteria" — video is supplementary at best.

3. **Wiki format.** GitHub wikis are searchable and editable, but they're separate from the repository. The judges will clone the repo and look at files. Putting docs in `docs/competition/` means they're part of the repo and visible in the file tree.
