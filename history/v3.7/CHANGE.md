# v3.7 — Color Detection: RGB to HSV

## What Changed
The WRO 2026 rulebook specifies pillar colors with exact RGB values. For example, the red pillar is defined as RGB(200, 30, 30), the blue pillar is RGB(30, 30, 200), etc. But under the track lighting (fluorescent tubes at unknown color temperature, probably ~4000K), these RGB values shift. Direct RGB matching is fragile.

`color_detect.py` converts the four rulebook pillar colors (red, blue, yellow, green) into HSV ranges, then applies per-pixel classification to a camera frame. The output is a mask image for each color, with white pixels indicating a match. The masks will be passed to blob detection (v3.8) for finding pillar locations.

HSV (Hue, Saturation, Value) is used because:
1. Hue is invariant to lighting intensity changes (brightness doesn't change the hue).
2. Saturation separates colored objects from white/gray walls.
3. Value captures brightness but is mostly ignored in thresholding.

The conversion process:
1. Take the rulebook RGB values and convert to HSV using OpenCV's `cv2.cvtColor()`.
2. Add a tolerance range (±10 hue, ±40 saturation, ±40 value).
3. Convert camera frames from BGR (OpenCV default) or RGB (PiCamera2) to HSV.
4. Apply `cv2.inRange()` for each color range, producing a binary mask.

## Why
Without color detection, the camera captures a raw image but the robot can't distinguish between a red pillar and a blue pillar. The WRO task requires specific interactions with each color: red pillars might be "touch and stop," blue pillars "go around on the right," etc. Color is the robot's primary way to understand what it's looking at.

We need robust color detection because:
- Track lighting varies between practice and competition venues.
- The pillars are small (50 mm diameter) and may be partially shadowed.
- The robot's own shadow may fall on the pillar.
- The camera's white balance may shift.

## Errors Encountered

### Red Spans Hue 0-10 AND 170-180 (Hue Wrap-Around)
This was the most frustrating bug. Red pillars were detected about 50% of the time. Sometimes they were detected perfectly, other times not at all. We spent hours adjusting the HSV ranges before realizing the problem.

In OpenCV, HSV hue ranges from 0 to 179. Red is centered around hue 0 (or 180, same thing). When we set a range like `lower_red = (160, 50, 50)` and `upper_red = (10, 255, 255)`, `cv2.inRange()` fails because upper < lower. OpenCV doesn't handle circular ranges.

```
WARNING: Red mask: 0 pixels detected (pillar is clearly red in frame)
WARNING: Red mask after adjusting range to (0,50,50)-(10,255,255): 1200 pixels (good)
WARNING: Red mask for darker red pillar (hue ~175): 0 pixels (missed)
ERROR: Red pillar detection rate: 52%
```

**Fix:** Detect red in two separate hue ranges and combine the masks with a bitwise OR. Range 1 covers hue 0-10 (the "warm" side of red), range 2 covers hue 170-180 (the "cool" side of red). We create two masks and OR them together.

```python
# Red: two ranges to handle hue wrap-around
lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)
```

After this fix, red detection rate went from 52% to 94%. The remaining 6% misses are due to extreme lighting conditions.

### Rulebook RGB vs. Real-World HSV Mismatch
The rulebook says "Green: RGB(30, 200, 30)". We converted this to HSV: [60, 170, 200] (in OpenCV: [60, 170, 200] → hue 60 = green). But on the actual track, the green pillar looked more yellow-green, measuring around hue 50-55. The rulebook RGB values are clearly calibrated to a different color space or lighting condition.

```
ERROR: Green mask: 0 pixels (rulebook green pillar in frame)
ERROR: Manual HSV pick: green pillar hue = 52 (expected 60)
```

**Fix:** We don't trust the rulebook RGB values. Instead, we run a calibration step: place each pillar under the track lighting, capture a frame, and manually sample the HSV values using a trackbar UI. We save the measured ranges to a JSON file. The calibration script runs once per venue.

```python
# Instead of hardcoding from rulebook
color_ranges = {
    "red": [(0, 50, 50), (10, 255, 255), (170, 50, 50), (180, 255, 255)],
    "blue": [(100, 50, 50), (130, 255, 255)],
    "yellow": [(20, 50, 50), (35, 255, 255)],
    "green": [(40, 50, 50), (60, 255, 255)],
}
# Loaded from calibration file
color_ranges = json.load(open("color_calib.json"))
```

### False Positives From Floor
White track floor with texture sometimes produced hue values that fell into the yellow or green range (the track has subtle green tint from fluorescent lighting reflection).

```
WARNING: Yellow mask: 3400 pixels (no yellow pillar visible!)
WARNING: Floor is white, detected as yellow
```

**Fix:** Add a minimum saturation threshold (S > 40) and minimum value threshold (V > 40). The track floor is desaturated (S < 30), while colored pillars are highly saturated. This eliminated 99% of floor false positives.

## Alternatives Considered
- **Color checker calibration**: Use a X-Rite ColorChecker passport, photograph it, and compute a color correction matrix. Overkill for 4 colors.
- **Machine learning**: Train a tiny CNN (MobileNetV1) to classify pillar colors. We prototyped this in TensorFlow Lite, but it added 200 MB to the deployment size and inference took 15 ms per frame. HSV thresholding takes < 1 ms.
- **LAB color space**: CIELAB is perceptually uniform and might be more robust. But OpenCV's LAB conversion is slower, and we already get good results with HSV.

## Current Status
`color_detect.py` produces binary masks for red, blue, yellow, and green pillars at 60 fps. Red detection uses the dual-range wrap-around fix. Color ranges are loaded from a calibration file rather than hardcoded from the rulebook. The masks are accurate enough for blob detection (v3.8) to find pillar centroids.

Next version (v3.8): Find colored blobs in the mask images, filter by size and aspect ratio, and report pillar positions.
