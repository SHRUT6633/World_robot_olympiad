# v4.7 — Pillar Distance Estimation

## What I Tried

Pillar detection (v4.4, v4.5) gives us bounding boxes in pixel coordinates. To plan a path around a pillar, we need its real-world distance from the robot. I wrote `pillar_dist.py` using the pinhole camera model:

```
distance = (real_height * focal_length) / pixel_height
```

Where:
- `real_height` = 100 mm (specified by WRO rules for all pillars).
- `focal_length` = camera focal length in pixels (from calibration).
- `pixel_height` = height of the pillar's bounding box in pixels.

```python
def estimate_distance(pixel_height, focal_length_px=520.0):
    return (100.0 * focal_length_px) / pixel_height
```

## The Error — Camera Angle Dependency

The estimate was wildly inaccurate. For a pillar at 1000 mm, I got readings from 600 mm to 1400 mm depending on the robot's pitch angle.

```
[PILLAR] [5.231] Pillar at center (320, 240), pixel_h=52px → distance=1000mm
[PILLAR] [5.432] (robot accelerated, nose pitched up)
[PILLAR] [5.433] Pillar at center (320, 200), pixel_h=42px → distance=1238mm
[PILLAR] [5.635] (robot braked, nose pitched down)
[PILLAR] [5.636] Pillar at center (320, 280), pixel_h=68px → distance=765mm
```

The actual pillar distance hadn't changed (robot was stationary, accelerating/braking in place — this was a bench test). The pitch from motor torque was compressing or stretching the pillar's apparent height.

Without pitch correction, the distance estimate is useless for navigation.

## What I Changed

I added **pitch angle correction from the IMU**. When the camera is tilted, the apparent pixel height of the pillar is foreshortened by `cos(pitch)`. The corrected formula:

```python
def estimate_distance(pixel_height, pitch_rad, focal_length_px=520.0, real_height_mm=100.0):
    corrected_height = pixel_height / math.cos(pitch_rad)
    return (real_height_mm * focal_length_px) / corrected_height
```

But this alone wasn't enough — the pillar also shifts vertically in the frame. I added a check that the pillar's bottom y-coordinate is within a reasonable range for the floor plane. If the bottom of the bounding box is too high (above the horizon line), the pillar is likely far away and the pixel height measurement is unreliable anyway.

```python
def is_reliable(bottom_y, pitch_rad, frame_height=480):
    horizon_y = frame_height / 2 + pitch_rad * focal_length_px
    return bottom_y > horizon_y
```

## Alternatives Considered

- **Stereo vision**: Two cameras would give direct depth via disparity. We don't have a second camera, and mounting one precisely is mechanical work we don't have time for.
- **ToF sensors for distance**: The VL53L1X ToF sensors are side-mounted and can't see forward pillars easily. We could add a forward-facing ToF, but the beam divergence (25°) means at 1 m the spot is ~450 mm wide — too large to distinguish individual pillars.
- **Machine learning depth estimation (MiDaS)**: State-of-the-art monocular depth estimation. Runs at ~2 fps on RPi 4. Not real-time enough.

## Still Broken

- **Occluded base**: If the pillar's base is hidden behind a wall or another pillar, the measured pixel height is too small and the distance is overestimated. No good fix without knowing the occlusion.
- **Roll angle**: If the robot is tilted sideways (roll), the pillar appears tilted in the frame. The bounding box height is still roughly correct (pillar is vertical), but the bottom contact point shifts. Roll correction is next if this becomes an issue.

## Lesson Learned

Any metric estimation from camera pixels must account for the camera's orientation. The pinhole formula assumes the camera is perpendicular to the object. In a moving robot with suspension flex and motor torque, that assumption never holds. IMU correction is not optional — it's the difference between a useless estimate and a usable one.
