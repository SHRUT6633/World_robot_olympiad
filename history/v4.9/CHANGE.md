# v4.9 — Visual Odometry Test

## What I Tried

We need a way to estimate robot motion without relying solely on wheel encoders (which slip on the smooth arena floor). Visual odometry estimates camera motion from optical flow between consecutive frames. I wrote `visual_odometry.py` to compute the robot's translational and rotational velocity from feature point tracks.

The pipeline:
1. Detect corners (Shi-Tomasi) in frame N.
2. Track them to frame N+1 using Lucas-Kanade optical flow.
3. Compute the fundamental matrix or use the estimated homography to extract rotation and translation.
4. Accumulate position.

```python
def process(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if self.prev_gray is not None:
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
        )
        # ... estimate motion from flow
    self.prev_gray = gray
```

## The Error — Too Slow for Real-Time

The robot's main control loop runs at 30 Hz. The visual odometry loop, even at 640×480, was running at **5 fps**:

```
[VO] [0.000] Frame 0: optical flow computed in 185 ms
[VO] [0.185] Frame 1: optical flow computed in 192 ms
[VO] [0.377] Frame 2: optical flow computed in 188 ms
[WARN] [0.565] Control loop running at ~5.3 fps — below minimum 15 fps
```

The bottleneck was `cv2.calcOpticalFlowFarneback` — it's a dense optical flow algorithm that computes flow for *every pixel*. On a 640×480 image (307,200 pixels), this takes ~180 ms. Even the sparse Lucas-Kanade method (`cv2.calcOpticalFlowPyrLK`) was slow at full resolution because of the pyramid construction overhead.

The robot can't run a 5 fps odometry loop — it would miss obstacles and fail to correct course in time.

## What I Changed

I made two changes:

1. **Reduced resolution to 320×240**. Downscaling the frame by 2× reduces pixel count by 4×, and the flow computation time drops by roughly 4×.
2. **Switched to FAST corner detector + Lucas-Kanade optical flow** instead of dense Farneback.

```python
fast = cv2.FastFeatureDetector_create(threshold=20)
keypoints = fast.detect(gray, None)
pts = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, pts, None)
```

FAST (Features from Accelerated Segment Test) is extremely fast — it detects corners by comparing only 16 pixels around each candidate. On a 320×240 image, it finds ~200-500 corners in under 2 ms. Lucas-Kanade then tracks only those sparse points, taking another 5-10 ms.

Total: ~15 ms per frame → **~60 fps** theoretical throughput. In practice, with image capture and pre-processing overhead, we get about 25-30 fps, which is comfortably above the 15 fps minimum.

## Alternatives Considered

- **ORB-SLAM**: Full SLAM system that does visual odometry plus loop closure. Requires significant CPU. On RPi 4, it runs at ~8 fps. Not worth the complexity for a known track — we don't need mapping, just motion estimation.
- **Optical flow on GPU**: The RPi 4's VideoCore GPU can't run OpenCV's CUDA modules. No GPU acceleration available.
- **Downsample more (160×120)**: This would be faster but loses too much detail — pillars at 2 m distance become ~8 pixels tall and are undetectable. 320×240 is the sweet spot.
- **IMU-only odometry**: Integrating gyro + accelerometer for position. The double integration of acceleration drifts quadratically — position error would be meters after 30 seconds.

## Still Broken

- **Featureless floor**: The track floor is a uniform grey. FAST finds very few corners on it. Most features come from the track edges, lane lines, and pillars. If the robot is in the middle of a straight with no nearby lines, visual odometry fails. We fuse with wheel encoder odometry for those situations.
- **Scale ambiguity**: Monocular visual odometry can't determine absolute scale — it only knows direction, not distance. We estimate scale from the known camera height above the floor, but this is noisy (±15% error in practice).

## Lesson Learned

Algorithm choice matters more than code optimisation. Farneback dense flow at 640×480 was always going to be too slow on a Raspberry Pi. Switching to FAST + sparse Lucas-Kanade at 320×240 wasn't a marginal improvement — it was a 10× speedup. Always profile the bottleneck before optimising, but also think about whether a different algorithm altogether would be faster.
