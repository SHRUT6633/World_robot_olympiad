# v4.4 — Red Pillar Detection

## What I Tried

Rule 13.21 of the WRO 2026 regulations specifies that red pillars on the track are exactly RGB(238, 39, 55). I wrote `red_pillar.py` to detect these pillars using colour thresholding in RGB space.

The approach was:

1. Convert frame to RGB (OpenCV uses BGR by default).
2. Define a colour mask: `cv2.inRange(frame, lower_red, upper_red)`.
3. Erode/dilate to remove noise.
4. Find contours and draw bounding boxes around anything large enough.

The colour range was generous to account for lighting variations:

```python
lower_red = np.array([200, 20, 30])   # R low, G low, B low
upper_red = np.array([255, 80, 80])   # R high, G high, B high
```

## The Error — Red Tape on Floor

On the first test run, the robot detected 4 red pillars. There were only 2 on the track. The extra detections were from red electrical tape used to mark inspection points on the arena floor.

```
[DETECT] [4.221] Red pillar at (312, 240), area=1560px, confidence=0.87
[DETECT] [4.320] Red pillar at (298, 430), area=920px, confidence=0.81
[WARN]   [4.321] Wait — that's below the bumper line. That's on the floor.
```

The second detection was at y=430 in the frame — well below where the floor starts. The red tape was a strip about 20 mm wide, and from the camera's perspective it occupied a similar pixel area to a distant red pillar.

I tried tightening the colour range but that caused genuine pillars to be missed when the overhead lights cast a shadow on part of the pillar. The floor tape was really close to the target colour (it was specified by the competition organisers as "inspection grade red").

## What I Changed

The key insight: pillars are **tall**, floor markings are **flat**. I added a minimum height-to-width ratio requirement:

```python
h, w = rect[1]
aspect = h / w if w > 0 else 0
if aspect < 1.5:
    reject
```

A red pillar is about 100 mm tall and ~30 mm in diameter, so its bounding box should be at least 3:1. Even from an angle, it shouldn't drop below 1.5:1. Red tape on the floor has an aspect ratio close to 0.2-0.5 (wide and short).

This immediately eliminated all false positives from floor markings.

## Alternatives Considered

- **Depth filtering**: If we had a depth map, we could reject anything at floor level. We don't have depth, but the aspect ratio proxy works surprisingly well.
- **Motion filtering**: The tape on the floor moves with the robot's perspective (looks like it's sliding), while pillars stay stationary. This could work but adds complexity.
- **Region-of-interest cropping**: I could crop the bottom 30% of the frame (where the floor is) and ignore detections there. But a close pillar *should* appear at the bottom of the frame, so this would miss nearby pillars.

## Still Broken

- **Red LEDs on other robots**: Another team's robot has red status LEDs. From a distance, the LED blob looks like a small red pillar. We can't filter by aspect ratio because the LED cluster could be circular. For now, we'll treat any red detection as potential pillar and use distance estimation (v4.7) to verify.
- **Red team uniforms**: If a team member stands near the track wearing a red shirt, it gets detected. Rule 13.21 only applies to pillars, but the robot doesn't know that.

## Lesson Learned

Single-channel colour thresholding is fragile. Combine it with geometric constraints (aspect ratio, position in frame, temporal consistency) before trusting a detection. A pillar isn't just a colour — it's a tall coloured object.
