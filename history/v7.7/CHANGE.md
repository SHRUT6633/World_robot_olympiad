# v7.7 — Parking State Machine

## Diary Entry — 2026-03-26

Parking is the final challenge of a WRO run. After completing all laps, the robot must navigate into a designated parking zone between two markers and stop. This is a complex multi-step maneuver that needs its own state machine.

Today I built `park_sm.py` — a 7-state parking sequence that guides the robot from "I just finished my last lap" to "I am parked and verified."

## The 7 states

The parking sequence is:

1. **IDLE** — waiting for the lap counter to signal "all laps done"
2. **MARKER_SEEN** — the robot has spotted the parking markers (ARUCO markers or visual landmarks)
3. **BETWEEN_MARKERS** — the robot has positioned itself between the two markers
4. **ALIGNING** — fine-tuning orientation parallel to the markers
5. **BACKING_IN** — reversing into the parking spot
6. **PARKED** — stopped in position, motors off
7. **VERIFIED** — sensors confirm parking position is valid

The flow is: IDLE → MARKER_SEEN → BETWEEN_MARKERS → ALIGNING → BACKING_IN → PARKED → VERIFIED.

## The alignment bug

State 4 (ALIGNING) uses the left and right time-of-flight sensors to check that the robot is parallel to the parking markers. The idea: when the robot is parallel, both ToF sensors read the same distance.

```python
def _check_alignment(self, tof_left, tof_right):
    diff = abs(tof_left - tof_right)
    return diff < self.alignment_tolerance
```

Tolerance was set to 2cm. In testing, the alignment check consistently failed:

```
[WARN] Alignment check failed: left=0.32m, right=0.28m, diff=0.04m
[WARN] Alignment check failed: left=0.31m, right=0.29m, diff=0.02m
[WARN] Alignment check failed: left=0.33m, right=0.27m, diff=0.06m
```

The left and right ToF sensors were reading different distances even when the robot was visually parallel. I checked the mounting — the sensors were physically aligned. What was going on?

The issue is that ToF sensors measure distance to the nearest surface in their field of view. The parking markers (walls/flat surfaces) might not be perfectly perpendicular to the sensor beam. At a slight angle, the ToF reading can be off by several centimeters because the IR beam reflects off the surface at an angle.

More precisely: the VL53L1X ToF sensors we're using have a field of view of about 25°. If the wall is at a 10° angle relative to the sensor, the beam hits the wall at a 10° angle, and the distance reading becomes:

```
actual_perpendicular_distance * cos(10°) ≈ distance * 0.985
```

That's only 1.5% error — not enough to explain 4cm differences. But combined with the sensor's own noise (±3cm at 1m range), and the fact that one sensor might be pointed at a slightly different surface (e.g., a marker post vs. a wall), the readings can easily differ by 4-6cm.

## The fix: rolling average filter

Instead of comparing single readings, I buffer the last 3 readings from each sensor and use their averages:

```python
def _update_readings(self, tof_left, tof_right):
    self._left_buffer.append(tof_left)
    self._right_buffer.append(tof_right)
    if len(self._left_buffer) > self.buffer_size:
        self._left_buffer.pop(0)
    if len(self._right_buffer) > self.buffer_size:
        self._right_buffer.pop(0)
    self._avg_left = sum(self._left_buffer) / len(self._left_buffer)
    self._avg_right = sum(self._right_buffer) / len(self._right_buffer)
```

And the alignment check uses the averaged values:

```python
def _check_alignment(self):
    diff = abs(self._avg_left - self._avg_right)
    return diff < self.alignment_tolerance
```

This smoothed out the noise. The alignment check now passes consistently. But I also had another issue: the tolerance was too tight. With the averaging, I could reduce the tolerance... but actually, I increased it from 2cm to 3cm because even with averaging, slight angle variations during the approach cause real differences. The 3cm tolerance accounts for the robot being "close enough" to parallel.

## The "BETWEEN_MARKERS" transition

Another bug I hit: the robot would enter MARKER_SEEN, approach the markers, then immediately transition to ALIGNING without ever properly centering BETWEEN_MARKERS. The transition logic was:

```python
if self._both_markers_visible:
    self.transition(ParkState.BETWEEN_MARKERS)
```

But `_both_markers_visible` was true as soon as the robot detected both markers, even if they were at the edge of the camera's field of view. The robot would start aligning while still off to one side.

I added a centering requirement: both markers must be within a "centered" zone (±15° from center of camera frame) before the transition:

```python
def _check_centered(self, marker_positions):
    for pos in marker_positions:
        angle = self._pixel_to_angle(pos.x)
        if abs(angle) > math.radians(15):
            return False
    return True
```

Now the robot properly centers itself between the markers before starting alignment.

## Alternatives considered

**Alternative 1: Single-stage parking (just drive in).** Simpler but unreliable — without alignment, the robot might hit a marker or overshoot the space.

**Alternative 2: Open-loop parking sequence.** Execute a fixed sequence of movements. Works if the robot always approaches from the same angle, but doesn't handle variations in approach.

**Alternative 3: Vision-only parking.** Use the camera for all positioning. Too slow — the camera runs at 15 FPS, and parking requires precise movements.

**Alternative 4: Multi-state with ToF averaging (chosen).** Combines camera for marker detection with ToF for fine alignment. The rolling average eliminates sensor noise issues.

## Testing

I ran 50 parking simulations with random approach angles. The multi-state sequence completed successfully in 48/50 attempts. The 2 failures were due to the robot approaching from too extreme an angle (>30° offset), which I've documented as a known limitation.

## Stats

- Lines of code: 223 (park_sm.py)
- States: 7
- ToF buffer size: 3 readings
- Alignment tolerance: 3cm (averaged)
- Success rate: 96%

Parking is one of the hardest parts of the WRO challenge. This state machine gives us a reliable, debuggable sequence that handles the most common approach scenarios.

— 2026-03-26, signing off.
