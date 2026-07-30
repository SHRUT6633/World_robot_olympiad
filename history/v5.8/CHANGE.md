# v5.8 — Cross-Sensor Verification

**Theme:** "Does this measurement make sense?"

We have multiple sensors: ToF (time-of-flight ranging), camera (visual detection of landmarks), and wheel encoders. They all provide position information, but they disagree — sometimes systematically.

Cross-sensor verification checks if two independent sensors agree on a measurement before accepting it into the filter. The principle: if the ToF says the wall is 1.2m away and the camera says the same wall is 0.8m away, at least one sensor is wrong. Reject both until they agree within tolerance.

I implemented `cross_verify.py` focusing on ToF vs camera distance checks. The setup: when the robot is near a wall, the ToF sensor measures perpendicular distance. The camera sees colored tape marks on the wall and estimates distance based on apparent size in the image.

The first real test: a simple wall approach.

```
[CROSS] ToF distance: 1.21m
[CROSS] Camera distance: 0.76m
[CROSS] Discrepancy: 0.45m — EXCEEDS TOLERANCE (0.10m)
[CROSS] Verdict: MISMATCH — rejecting both
```

Wait, what? A 45cm discrepancy? That's not noise — that's a systematic error. I investigated.

The camera was mounted 5cm to the left of the ToF sensor, and the camera's optical axis was not perfectly parallel to the ToF beam. At 1m range, a 3° angular misalignment produces 5cm of lateral offset, which for a wall-normal measurement translates to about 5cm error in perpendicular distance. But we were seeing 45cm.

The real issue: the camera was detecting the wrong landmark. It was picking up a reflection from a glossy floor surface, seeing the tape mark at half the actual distance. Fixing the landmark detection algorithm (better thresholding) solved most of the discrepancy.

But 5cm of systematic offset remained. That's the physical misalignment between the camera and ToF sensor frames. The ToF beam originates at the front bumper; the camera is on the mast 5cm back. When the robot is angled to the wall, the difference in mounting positions produces a trigonometric offset.

The fix: calibrate the camera-to-sensor transform. I measured the physical offset between the ToF and camera frames:

- x_offset: 0.05m (camera is 5cm behind ToF)
- y_offset: 0.00m (laterally aligned)
- yaw_offset: 0.5° (tiny rotation from mounting)

I added a transform that projects the ToF measurement into the camera frame (or vice versa) before comparison:

```python
def tof_in_camera_frame(tof_distance, robot_heading, wall_angle):
    # Project ToF point into camera coordinates
    dx = x_offset * cos(robot_heading) + tof_distance * sin(wall_angle)
    camera_distance = sqrt(dx**2 + tof_distance**2 + 2*dx*tof_distance*cos(wall_angle))
    return camera_distance
```

After calibration, discrepancies dropped below 3cm in normal operation.

The cross-verification logic:
1. Measure wall distance with ToF
2. Measure wall distance with camera (landmark apparent size)
3. Transform ToF measurement into camera frame
4. If |d_tof - d_camera| < tolerance (0.10m), accept both (weighted average)
5. If discrepancy is large, check which sensor has lower variance and prefer it
6. If discrepancy is extreme (>0.25m), reject both and use motion model only

I also added per-sensor health metrics. A sensor that consistently fails cross-verification gets its measurement noise R inflated, effectively reducing its influence until it starts agreeing again.

Testing results:
- Normal conditions: cross-verify passes 95% of checks
- Glossy floor (camera reflection): cross-verify correctly rejects camera, uses ToF
- Low battery (ToF noise increases): cross-verify rejects ToF, uses camera
- Both sensors disagree: filter relies on motion model briefly, re-checks next cycle

Key files:
- `cross_verify.py` — Cross-sensor verification logic
- `tof_camera_calib.py` — Calibration script for sensor transform
