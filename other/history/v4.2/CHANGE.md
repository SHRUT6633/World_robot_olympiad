# v4.2 — Free Space Detection

## What I Tried

The robot needs to know where it can drive. Lane lines tell it where the track boundaries are, but obstacles (pillars, walls, other robots) can occupy the drivable area. I wrote `free_space.py` to classify every pixel in the camera frame as either "drivable" (floor) or "obstacle".

My first approach was simple: grayscale the image, apply a threshold to find bright (floor) vs dark (obstacle) regions, then run connected components to find the largest contiguous drivable area.

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
```

The arena floor is a uniform light grey, so thresholding seemed like a reasonable first pass. I'd then compute a "free space score" as the percentage of pixels classified as floor in a horizontal strip in front of the robot.

## The Error — Shadows Classified as Obstacles

The competition hall has overhead lights, and the robot casts a shadow directly in front of itself. Additionally, the walls cast a shadow where they meet the floor.

When I ran the robot, it would drive straight for about a metre, then stop and report "no free space". Looking at the debug video, the shadow of the robot's own body fell across the floor in the bottom of the frame, and the thresholding classified that dark region as an obstacle.

```
[FREE_SPACE] [3.142] Free space: 12% — too low, stopping
[DEBUG] [3.142] Frame mean brightness: 97 — well below threshold
```

The whole frame looked dimmer than expected because the camera exposure was compensating for the bright arena lights, making the floor actually darker than 160 in grayscale.

I tried lowering the threshold, but that made the problem worse: lighter shadows got classified as floor, and darker floor areas got classified as obstacles. The grayscale alone simply doesn't carry enough information to separate shadow from obstacle.

## What I Changed

I switched from grayscale thresholding to **HSV colour space, using the saturation channel**.

The key insight: shadows are a *brightness* change, not a *colour* change. In HSV, a shadow reduces the Value channel but leaves Hue and Saturation mostly unchanged. Obstacles (pillars, walls) change all three.

The new pipeline uses a texture-based approach: I compute the standard deviation of saturation in a 16×16 window. Floor (even shadowed floor) is smooth (low std dev). Obstacles are textured (high std dev).

```python
sat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 1]
texture = cv2.boxFilter(sat, ddepth=-1, ksize=(16, 16), normalize=False)
```

This worked much better. The shadowed floor near the robot is classified as drivable, while the white wall and coloured pillars are correctly classified as obstacles.

## Alternatives Considered

- **Deep learning (MobileNet-SSD)**: Too slow on RPi 4. We need ~15 fps, and even MobileNet runs at ~8 fps on the CPU.
- **Depth camera (Intel RealSense)**: Would make this trivial (anything above floor plane = obstacle), but we don't have one and the competition budget doesn't allow it.
- **Structured light**: A line laser + camera would give us a single scan line of depth. Possible future upgrade if we have time.

## Still Broken

- **Floor reflections**: On polished floor sections, the reflection of a red pillar looks like a red patch on the floor. The texture feature sees it as an obstacle. I'll need to address this when we do colour pillar detection.
- **Dynamic exposure**: The camera auto-exposure adjusts when the robot turns towards a bright wall, changing the saturation values. I should lock exposure once calibrated.

## Lesson Learned

Grayscale thresholding is not a segmentation method. For any real-world environment with shadows, you need either colour information, depth, or machine learning. In our case, texture from HSV saturation gave us a free lunch — no ML, no extra hardware — and solved the shadow problem completely.
