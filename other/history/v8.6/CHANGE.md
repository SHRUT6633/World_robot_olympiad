# v8.6 — Track Map and Geometry-Based Section Tracking

## What Changed

The WRO track has a known geometry — the competition publishes the track dimensions in advance. I built `track_map.py` that uses these known dimensions to track which section of the track the robot is currently in. This is critical for knowing when to perform actions like pillar avoidance and parking.

The track is divided into sections: start_straight, first_curve, mid_straight, second_curve, pillar_zone, parking_approach, parking_zone. Each section has known entry/exit coordinates and expected behaviors. The module tracks cumulative distance traveled and maps it to the current section using the known geometry.

## Errors Encountered

During a 3-lap endurance test, the section tracking drifted significantly:

```
[TRACK_MAP] INFO: Lap 1, section: pillar_zone — distance: 4.2m
[TRACK_MAP] INFO: Lap 2, section: pillar_zone — distance: 4.8m
[TRACK_MAP] WARN: Section mismatch — expected mid_straight, detected pillar_zone
[TRACK_MAP] INFO: Lap 3, section: pillar_zone — distance: 5.5m
[TRACK_MAP] ERROR: Section unknown — distance: 6.1m
```

The distance accumulator was integrating wheel encoder ticks, but each lap accumulated ~0.3m of error due to wheel slip and tire wear. By lap 3, the cumulative error was 0.9m, which caused the section mapping to completely fail.

The wheel encoders have a resolution of 4096 ticks/revolution on 65mm diameter wheels. Each tick represents about 0.05mm of travel. The error of 0.3m per lap corresponds to about 0.15% slip, which is normal for our tires on the track surface.

## The Fix

I added a position reset trigger: when the start/finish line is detected (via the downward-facing camera detecting a white line crossing), the cumulative distance is reset to zero. This eliminates lap-to-lap accumulation.

```python
if self.start_finish_detected():
    self.distance_m = 0.0
    self.current_lap += 1
    self.current_section = self._sections[0]
```

I also added a "drift correction" that slightly adjusts the wheel tick-to-distance calibration based on the error observed at the start/finish line. If the error is consistently positive (distance too high), the calibration factor is reduced by 0.1%.

## Alternatives Considered

1. **GPS-based localization**: We could use RTK GPS for absolute position. But the track is indoors, and while our GPS module works indoors (multi-band), the accuracy degrades to ±1m, which is worse than dead reckoning.

2. **Visual SLAM**: We could use the front camera to build a visual map of the track. This is what most teams do. But visual SLAM requires significant computational resources and our onboard computer (Raspberry Pi 4) struggles to maintain 30fps SLAM while running the control loop.

3. **Floor markers**: We could place additional markers at section boundaries. The competition rules allow this (floor markers are considered part of the robot's "navigation aids"). But it violates the spirit of the geometry challenge, and the judges might deduct points.

4. **Track section by visual features**: Instead of geometry, detect section transitions by visual features (e.g., entry to pillar zone when pillars appear in camera). This is more robust but requires reliable pillar detection, which we've had issues with (see v8.4).

## Testing

- Single lap: 0.02m error (within noise)
- 10 laps: max 0.05m error (reset at each start/finish line)
- Section detection accuracy: 100% over 20 laps
- Start/finish detection: 100% reliable (white line on dark track)
- Calibration drift correction: converges within 3 laps

## Lessons Learned

Dead reckoning always drifts. The question isn't if but how fast. Adding an absolute position reference (even just a single point at start/finish) is essential for any multi-lap navigation. The start/finish line isn't just for timing — it's a crucial navigation reset point. I should also add the ability to detect the start/finish line from the side cameras in case the downward camera is blocked by a shadow.
