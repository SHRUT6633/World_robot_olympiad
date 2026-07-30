# Version 1.4 — The Polish

## What Changed

This version did not add any new hardware features. It made the existing
code better. Like cleaning your room before guests arrive.

Three big things:
1. Comments everywhere (for the judges)
2. Surprise Rule config (for the competition)
3. New perception modules (pillar tracking, parking, track map)

## Why Comments Matter

The judges score Documentation (Appendix C) for up to 30 points. That is
30% of the total score. They look at your code and ask: "Does this person
understand what they wrote?"

I did not write comments in v1.0 because I was in a hurry. By v1.4, I knew
the code would be evaluated. So I went file-by-file and added explanations.

Not just "what" the code does (that is obvious from reading the code).
But "WHY" it does it. For example:

```python
# We use OPPOSITE_PHASE steering in narrow tracks because
# it reduces the turning radius by ~40% compared to SAME_PHASE.
# This is critical for the 600mm wide section (Rule 13.8).
```

Without this comment, a judge might think we chose opposite-phase randomly.
With the comment, they see: "Ah, they know the rule and they have a reason."

## The Surprise Rule System

Every year at competition, the judges announce a surprise rule change.
Past surprises include: "Pillars are on opposite sides" and "Drive the
other direction" and "Reduced speed."

I thought: what if we can adapt without rewriting code?

**The idea:** A single YAML file (`config/surprise_rules.yaml`) with every
possible surprise mapped to a config value. When judges announce the rule,
we change ONE LINE and restart.

Example scenarios:

| Scenario | Config Change | Why |
|----------|--------------|-----|
| Pillars swapped | `pillar_logic: "REVERSED"` | Red pillars are now on the right, green on the left |
| Drive direction | `drive_direction: "CCW"` | Clockwise → counter-clockwise |
| Narrow track | `narrow_track: "ENABLED"` | Track is only 600mm wide, use opposite-phase steering |
| No stopping | `stop_and_go: "DISABLED"` | Robot should not stop at pillars |
| Speed limit | `max_speed_ms: 1.0` | Judges want slower driving |

**Why YAML and not a database or environment variables?**
YAML is human-readable. A judge can open the file and see all options.
No special tools needed. Just a text editor.

## Pillar Tracking (NEW)

v1.0 could detect pillars (red, green). But it did not remember which ones
it passed. The rules say: pass red on the LEFT, pass green on the RIGHT.
If you pass on the wrong side, you lose points.

So I wrote `PillarTracker`. It keeps a list of every pillar the robot has
passed, which side it was on, and whether it was correct or wrong.

At the end of the race, we know: "We passed 6 pillars. 4 correct. 2 wrong
(those were in lap 1 when the camera was calibration)."

## Parking Detection (NEW)

Parking is the hardest part. You have to stop between two magenta markers,
parallel to the wall, within 2cm, and stay still for 30 seconds.

I wrote `ParkingDetector` with a state machine:
1. **IDLE** — driving normally, looking for magenta
2. **MARKER_SEEN** — first magenta pillar detected
3. **BETWEEN_MARKERS** — second marker found, we are in the zone
4. **ALIGNING** — using left/right ToF sensors to measure distance to wall
5. **BACKING_IN** — reversing into the spot
6. **PARKED** — stopped, waiting for verification
7. **VERIFIED** — 30 seconds passed, judges can score

Each state has a timeout (fail-safe). If the robot gets stuck in ALIGNING
for more than 10 seconds, it tries anyway. Better to park poorly than to
time out.

**Why not use the camera for parking alignment?**
Because the camera is noisy at close range. The ToF sensors are millimeter-
accurate. We use `|left_tof - right_tof| ≤ 20mm` as the alignment check.

## Track Map (NEW)

I did not want to use SLAM. SLAM is heavy (uses a lot of CPU). Instead,
I used geometry. The track is a rectangle (3m × 1m outer, 1m × 1m inner,
from Rule 13.1-13.8). The centerline is (outer + inner) / 2. Each lap is
4 × centerline length.

`TrackMap` just tracks: how far have we gone? Which section are we in?
(straight, corner, obstacle zone). No mapping. No images. Just math on
distance.

This runs at 100 Hz with 0.1% CPU usage. SLAM would use 30%+.

## What I Would Do Differently

1. **Write comments from v1.0.** I wasted a whole day adding them later.
2. **Plan the config system earlier.** The surprise rule config was a
   late addition. It should have been there from the start.
3. **Test parking more.** The parking detector logic works on paper but
   we have not tested it on the actual robot yet.

## Real Talk

This version looks polished. The comments are clean. The config system
is clever. The code is organized.

But do you know how many bugs are still hiding? Probably dozens. We fixed
the ones we found, but there are always more. That is software.

Version 1.4 is not perfect. It is just the best we could do before we
ran out of time.
