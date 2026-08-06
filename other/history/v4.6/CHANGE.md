# v4.6 — Pink/Magenta Detection

## What I Tried

Rule 13.27 defines the parking marker colour as magenta, RGB(255, 0, 255). These markers are placed at parking spots on the track. The robot needs to detect them to know where to stop.

The markers are small plastic tabs mounted flush with the track surface. I wrote `pink_detect.py` using HSV colour thresholding (learning from v4.5's lessons).

```python
lower_magenta = np.array([140, 100, 100])
upper_magenta = np.array([170, 255, 255])
```

The hue range was set to capture magenta/pink hues (around 150° on the HSV wheel).

## The Error — Marker Too Small at Distance

The markers are only **20 mm tall**. From the camera mounted 200 mm above the floor, a 20 mm marker at 1 metre distance occupies only about 10-15 pixels in a 640×480 frame. That's barely a blob.

The first test run:

```
[PINK] [12.301] Magenta candidate at (320, 400), area=45px — rejecting, area < 400
[PINK] [12.401] Magenta candidate at (318, 398), area=38px — rejecting
```

The detector was correct — there was a magenta marker there — but the contour area was far below the `MIN_AREA = 400` threshold I'd carried over from pillar detection. When I lowered the threshold to `MIN_AREA = 30`, I got false positives from pink specular reflections on the floor and from a pink sticker on the competition judge's clipboard.

```
[PINK] [15.002] Magenta at (160, 450), area=55px — ACCEPTED
[PINK] [15.003] Magenta at (480, 440), area=62px — ACCEPTED
[ERROR] [15.004] Wait, there should only be one marker in this zone.
```

The false positive was a reflection of the overhead "EXIT" sign (which has a pinkish tint) on a polished floor section.

## What I Changed

I added a **distance gate**: only run the magenta detector when the robot is within 500 mm of a known parking location. This is determined by odometry from the wheel encoders.

```python
def process(self, frame, distance_to_target_mm):
    if distance_to_target_mm > 500:
        return []
    # ... colour detection as normal
```

Since the robot only needs to detect the parking marker when it's approaching a parking spot (last 500 mm), this dramatically reduces false positives. The 500 mm limit was chosen empirically: at this distance, the 20 mm marker occupies about 100 pixels, which is large enough for reliable contour detection.

I also increased the minimum area to 80 pixels when the distance gate is active, to reject noise:

```python
if area < min_area:
    continue
```

## Alternatives Considered

- **Zooming the camera digitally**: Crop the camera ROI to the forward area only. This increases effective resolution for the marker but would require recalibrating all other detectors.
- **Using a second downward-facing camera**: A small camera pointed straight down at the floor would see the marker clearly. However, we only have one camera port on the RPi.
- **Optical flow for marker confirmation**: If we see a magenta blob and it doesn't move relative to the floor as the robot approaches, it's real. This is interesting but complex for a 20 mm marker.

## Still Broken

- **Judge's clipboard**: The judges walk around with clipboards that have pink stickers. If a judge is standing near the parking zone, the robot might detect their clipboard. The 500 mm gate helps, but doesn't eliminate it. I'm adding a "recheck on next frame" rule: if the marker disappears from one frame to the next (judge moved), ignore it.
- **Marker wear**: At previous competitions, the magenta markers had faded after a day of use. The colour shifts towards pinkish-grey, which is harder to distinguish from floor reflections.

## Lesson Learned

Detection thresholds must scale with object size. A parameter like `MIN_AREA` that works for 100 mm pillars is wrong for 20 mm markers. Adding context-aware gates (distance, odometry position) makes the detector more robust than any amount of colour tuning.
