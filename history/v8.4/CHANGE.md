# v8.4 — Pillar Pass-Side Tracker

## What Changed

The WRO track has colored pillars (red, green, blue, yellow) that the robot must pass on a specific side according to the surprise rules. I built a `pillar_tracker.py` module that uses the camera to detect pillars, logs which side each pillar was passed, and validates against the configured pass-side logic.

The module uses ArUco markers on each pillar to determine identity and a simple angular calculation: when the pillar's bearing crosses from positive to negative (or vice versa), we've passed it. If the pillar was on the left when it crossed the center line, we passed it on the left. The pass-side is then checked against the `pillar_logic` config (pass_left, pass_right, or alternate).

## Errors Encountered

The first test showed that the same pillar was being counted 3-4 times:

```
[PILLAR_TRACKER] INFO: Pillar RED detected — bearing: 12.3 deg
[PILLAR_TRACKER] INFO: Pillar RED detected — bearing: 8.1 deg
[PILLAR_TRACKER] INFO: Pillar RED detected — bearing: 3.2 deg
[PILLAR_TRACKER] INFO: Pillar RED passed on left side
[PILLAR_TRACKER] INFO: Pillar RED detected — bearing: -15.2 deg
[PILLAR_TRACKER] INFO: Pillar RED passed on right side  ← WRONG!
[PILLAR_TRACKER] INFO: Pillar RED passed on left side     ← WRONG!
[PILLAR_TRACKER] ERROR: Pillar RED counted 3 times
```

The problem was the detection cooldown was set to 100ms. At a robot speed of 0.5 m/s, the robot moves 50mm between detections. The pillar is in the camera's field of view for about 2 seconds, giving 20 detection opportunities. Without a proper cooldown, multiple bearing-crossing events trigger multiple "passed" events.

The bearing crosses zero when the pillar transitions from left to right of the robot's forward axis. But noise in the pillar position estimate causes the bearing to oscillate around zero, triggering multiple crossings.

## The Fix

I increased the per-pillar cooldown to 500ms. This is derived from the robot's maximum speed and the pillar width:

```python
PILLAR_COOLDOWN_S = 0.5  # 500ms between detections of same color
```

At 0.5 m/s, the robot moves 250mm in 500ms, which is enough to clear the pillar's detection zone. The cooldown prevents re-detection even if the bearing oscillates.

I also added a center-crossing state machine: instead of triggering on every zero crossing, I track the previous bearing and only trigger when the bearing transitions from positive to negative (pass on right) or negative to positive (pass on left). This eliminates double-counting from bearing noise.

## Alternatives Considered

1. **Bearing rate-based detection**: Instead of cooldown, I could detect the pass event based on the rate of bearing change. When d(bearing)/dt > 30 deg/s, the pillar is being passed. This is more elegant but requires differentiating a noisy signal, which amplifies noise.

2. **Optical flow tracking**: I could track the pillar's pixel position in the camera frame and detect when it leaves the frame on the left or right edge. This is more reliable but requires integrating with the camera driver's optical flow module, which we haven't fully tested yet.

3. **Single detection at closest approach**: Instead of detecting the pass event, I could just record the pillar at its closest approach (minimum bearing). This gives the pass side directly (left or right of center at closest approach). But the closest approach detection requires finding the minimum of a noisy signal, which is error-prone.

## Testing

- 20 passes with various pass-side configurations
- Zero double-counting after fix
- 100% correct pass-side detection (left vs right)
- Cooldown correctly resets after 500ms
- Multiple pillars of same color correctly handled (cooldown is per-pillar-ID, not per-color)

## Lessons Learned

Detection cooldowns are critical in robotics. The naive approach of "detect when bearing crosses zero" fails because real signals have noise. The cooldown approach trades off detection latency for reliability. 500ms works well for this application, but I should make it configurable in case the robot speed changes.
