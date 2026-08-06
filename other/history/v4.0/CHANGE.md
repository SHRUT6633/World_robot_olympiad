# v4.0 — Lane Detection

## What I Tried

This version is all about getting the robot to see lane boundaries on the track. The WRO 2026 track has white lane lines on a dark surface, so I started with the classic computer vision pipeline: grayscale → Gaussian blur → Canny edge detection → Hough Transform → lane line extraction.

I wrote `lane_detect.py` as a standalone module. The idea was simple: grab a frame from the downward-facing camera, run it through OpenCV's HoughLinesP, get a set of line segments, and filter those into left and right lane boundaries based on slope and position.

The first pass used default parameters:

```python
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=40, maxLineGap=10)
```

Then I grouped lines by slope sign: negative slopes → left lane, positive slopes → right lane. For each group I fit a linear regression to get a single boundary line.

## The Error — Noisy Hough Lines

Immediately I saw that `HoughLinesP` returns *everything*. Every little edge fragment in the frame — floor scratches, shadows from overhead lighting, wheel marks, dust — became a "lane line". The output looked like a plate of spaghetti.

The actual log output was:

```
[WARN] [0.421] [lane_detect]: Computed left slope = -1.23, right slope = -0.45 ... that's both pointing left?
[WARN] [0.422] [lane_detect]: Lane width 42mm — impossible, rejecting frame
```

The noise was so bad that the left/right lane regression frequently produced two left-pointing lines, or lines with a "lane width" of a few centimetres. The robot had no idea where the track actually was.

## What I Changed

I added a **5-frame sliding window filter** and **slope validation**. The idea:

1. Collect Hough lines for 5 consecutive frames.
2. For each frame, compute the candidate left/right slopes.
3. Average the slopes across the window.
4. Reject any line whose slope falls outside the expected range for a forward-facing camera: left lane slope in `(-2.0, -0.2)`, right lane slope in `(0.2, 2.0)`.

The rejection logic:

```python
def slope_in_range(slope, side):
    if side == "left":
        return -2.0 < slope < -0.2
    elif side == "right":
        return 0.2 < slope < 2.0
    return False
```

If either line fails the range check, the whole frame is rejected and the robot uses the previous frame's lanes. This prevents the controller from getting garbage data.

## Alternatives Considered

- **Deep learning segmentation (U-Net)**: Overkill for lane lines. We don't have a GPU on the robot (Raspberry Pi 4), and even a tiny model would drop us below 10 fps.
- **Colour thresholding in HSV**: The lane lines are white, but so are the competition banners and the robot's own body in some frames. Edge detection is more robust to lighting changes.
- **RANSAC line fitting**: I considered replacing linear regression with RANSAC to handle outlier Hough lines. I may revisit this in v4.x if the averaging still produces jitter, but for now the sliding window cleans it up well enough.

## Still Broken

The lane detection still struggles in two cases:

1. **Sharp corners**: The lane lines curve, and a single straight-line model fails. The robot thinks the lane suddenly narrows or disappears. This will need to be addressed when we add corner detection in v4.3.
2. **Track edge vs. lane line**: On parts of the track where the lane line is worn or missing, the robot tries to track the edge of the track itself. This sometimes works, sometimes picks up a shadow.

## Lesson Learned

Don't trust raw Hough output. Computer vision on a cheap camera in a competition hall with variable lighting is fundamentally noisy. Averaging and sanity-checking every measurement is not optional — it's the bare minimum.

Also: tuning Hough parameters (`threshold`, `minLineLength`, `maxLineGap`) is *highly* sensitive to camera height and angle. I had to retune after mounting the camera 10 mm higher. Next time I'll calibrate these dynamically or at least document them clearly.
