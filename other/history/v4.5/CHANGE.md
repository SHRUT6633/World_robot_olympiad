# v4.5 — Green Pillar Detection

## What I Tried

Rule 13.22 defines green pillars as RGB(68, 214, 44). Following the same pattern as v4.4, I wrote `green_pillar.py` with colour thresholding in RGB space.

```python
lower_green = np.array([40, 180, 20])
upper_green = np.array([100, 255, 80])
```

Added the same aspect ratio filter (≥1.5:1) from the red pillar detector since it worked well.

## The Error — Green Merges with Floor

The test track has a green-coloured floor section. It's meant to simulate a "grass" zone. When the robot reached this section, everything was detected as a "green pillar":

```
[DETECT] [8.102] Green pillar at (160, 200), area=48000px, aspect=0.8
[DETECT] [8.103] Green pillar at (320, 240), area=51200px, aspect=0.9
[WARN]   [8.104] 47 green pillars detected — floor is green
```

The entire floor was being masked as green. The green floor's RGB values were approximately (60, 200, 40) — well within our detection range. The aspect ratio filter didn't help because the floor itself was a single connected "blob" with an aspect ratio close to 1.0 (the entire visible floor area).

I tried narrowing the RGB range, but then green pillars in shadow (e.g., near a wall) dropped out. The problem is that RGB thresholding is lighting-dependent, and the green channel dominates in both the pillar and the floor.

## What I Changed

I switched from RGB to **HSV colour space** and tuned the range at the competition venue.

The key advantage of HSV: Hue is invariant to lighting changes (to first order). A green pillar has a specific hue (~120° on the HSV wheel), while the green floor had a slightly different hue because it was a different material (paint vs. plastic).

Venue calibration tuning process:
1. Place robot 500 mm from a green pillar under arena lighting.
2. Sample the Hue value at the pillar's centre pixel.
3. Record H_min/H_max that captures the pillar but excludes the floor.
4. Rinse and repeat from different distances and angles.

The final values were surprisingly narrow:

```python
lower_green_hsv = np.array([50, 100, 100])
upper_green_hsv = np.array([80, 255, 255])
```

The H range of 50-80 selects only true green. The S minimum of 100 ensures we don't pick up desaturated green-grey floor sections. The V minimum of 100 excludes shadows.

## Alternatives Considered

- **Normalised RGB**: Divide each channel by `R+G+B`. This reduces lighting dependence but compresses the dynamic range and makes green less distinguishable from grey.
- **Colour checker calibration**: Use a known colour reference (X-Rite ColorChecker) to compute a colour correction matrix for the camera. Too complex for a robotics competition — the lighting changes between heats anyway.
- **Adaptive thresholding**: Use Otsu's method on the Hue channel to automatically find the green peak. This is actually promising and might replace manual tuning in v5.x.

## Still Broken

- **Green walls**: If the arena has green walls (some venues do), the robot will detect the entire wall as a green obstacle. Rule 13.22 specifically says "pillars", but the robot doesn't read rules.
- **No green pillars at all**: If there are zero green pillars on the track (possible in some configurations), the HSV range tuned at the venue may false-positive on a green sponsor logo. Need a minimum pillar height check — if the bounding box doesn't extend above a certain y-coordinate (indicating height above floor), reject.

## Lesson Learned

RGB colour thresholding is useless when the background shares a similar colour with the target. HSV is better because it isolates hue from illumination, but it still requires venue-specific tuning. Always bring a calibration target and budget 30 minutes at the competition venue to tune colour ranges.
