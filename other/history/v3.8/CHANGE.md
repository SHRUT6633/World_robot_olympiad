# v3.8 — Blob Detection

## What Changed
The color detection masks from v3.7 show which pixels match each pillar color, but they're noisy—there are isolated pixels, small speckles from reflections, and large regions that need to be grouped. We need to find contiguous blobs that correspond to actual pillars and reject everything else.

`blob_detect.py` takes the binary mask for each color, finds connected components (blobs) using `cv2.findContours()`, and filters them by:
- **Minimum area**: 200 square pixels. Smaller blobs are noise.
- **Aspect ratio**: Pillars are taller than wide (roughly 2:1 to 3:1 aspect ratio at typical viewing angles). Blobs with width > height are rejected (likely floor reflections or wall marks).
- **Solidity**: Ratio of blob area to convex hull area > 0.6. Pillars are roughly cylindrical—they appear as a filled shape, not a spiky outline.

The output is a list of `Blob` namedtuples: `(color, x, y, width, height, area)`. These are the pillar detections that the robot's control system will use for navigation decisions.

## Why
Raw masks are unusable for control. A mask might have 500 white pixels spread across 50 separate blobs. Without blob detection, we can't tell which blob is the pillar and which is a reflection. The robot would try to navigate toward a floor reflection, crash into the real pillar, and fail the WRO task.

Blob detection is the bridge between pixel-level color classification and object-level understanding. It converts "these 200 pixels are red" to "there's a red pillar at position (320, 240) in the image."

## Errors Encountered

### Reflections On Track Floor Create False Positives
The WRO track floor is glossy white. When a colored pillar is present, its reflection on the floor shows up in the binary mask as a second blob below the real pillar. The reflection has the same color, similar size, and passes both the minimum area and aspect ratio filters.

```
WARNING: 2 red blobs detected (expected 1)
WARNING: Blob 1 at (310, 180), area=450px (real pillar)
WARNING: Blob 2 at (312, 310), area=380px (floor reflection)
ERROR: Robot navigates toward reflection instead of real pillar
```

We initially tried to fix this by lowering the saturation threshold to exclude the reflection (reflections are less saturated because the glossy floor washes out color). But the reflection was still saturated enough to pass, especially under bright track lights.

**Fix:** Add an aspect ratio filter. Real pillars have a height-to-width ratio > 1.5 (taller than wide). Reflections have a ratio < 1.0 (wider than tall) because the reflection on the floor is foreshortened—it appears squashed horizontally. We reject any blob where `width > height`.

```python
if width > height:
    continue  # likely a floor reflection
```

We also considered a vertical position heuristic: the lower half of the image is "floor" and the upper half is "wall/pillar space." But this isn't robust—the camera angle changes if the robot tilts.

### Blob Merging When Pillars Are Close
When two pillars are close together (e.g., a red and blue pillar placed 100 mm apart per WRO rules), their color masks sometimes overlap, creating a single blob that covers both pillars. The blob's centroid is in the middle, between the two pillars, which is wrong.

```
WARNING: 1 yellow blob at (320, 240), area=900px
WARNING: 1 green blob at (310, 230), area=850px
WARNING: These should be 2 separate pillars
```

**Fix:** Apply morphological erosion to the mask before blob detection (kernel 3×3, 1 iteration). This separates blobs that are barely touching. Then dilate back to restore the original size. This is a standard "opening" operation (erosion + dilation) that removes small connections between nearby blobs.

```python
kernel = np.ones((3, 3), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
```

We also increased the minimum area threshold to 200 px (from 100 px) to remove small debris that was creating false blobs.

### Blob Detection Too Slow For 60 fps
The first version called `cv2.findContours()` on all 4 masks sequentially, then processed each contour. At 60 fps, this took about 25 ms total (6 ms per mask), leaving only 16 ms for other processing. The frame rate dropped to about 30 fps.

```
PERF: Frame processing time: 28 ms (target: 16.7 ms for 60 fps)
PERF: Actual frame rate: 35 fps
```

**Fix:** Downscale the mask images by 2x before contour finding. The masks are already binary (1 bit per pixel), so downscaling loses little information but halves the contour search space. We also parallelize the 4 color channels using Python's `concurrent.futures.ThreadPoolExecutor`.

```python
scale = 0.5
small_mask = cv2.resize(mask, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
contours, _ = cv2.findContours(small_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Scale coordinates back up
x, y, w, h = cv2.boundingRect(contour)
x, y, w, h = int(x/scale), int(y/scale), int(w/scale), int(h/scale)
```

After optimization: 8 ms total for all 4 colors. Frame rate back to 60 fps.

## Alternatives Considered
- **SimpleBlobDetector**: OpenCV's built-in `cv2.SimpleBlobDetector` provides blob filtering by area, circularity, inertia, etc. It works well but provides less control over aspect ratio filtering. We use custom code for that reason.
- **Watershed segmentation**: For separating touching blobs. Overkill—erosion+opening works fine for the typical spacing between WRO pillars.
- **YOLO object detection**: A neural network could detect pillars with bounding boxes in one forward pass. But training requires labeled data, and inference adds latency. HSV+blob detection is lighter and faster.
- **Find pixel centroid**: Instead of contour finding, we could simply find the centroid of all white pixels in the mask. This is faster but doesn't allow size filtering.

## Current Status
`blob_detect.py` finds pillar blobs at 60 fps. False positives from floor reflections are rejected by the aspect ratio check. Nearby pillars are separated by morphological opening. The output is a list of detected pillars with color, position, and size. This is the highest-level perception output so far—the control system now knows where pillars are and what color they are.

Next version (v3.9): Sensor health monitoring — track sensor failure rates and disable malfunctioning sensors gracefully.
