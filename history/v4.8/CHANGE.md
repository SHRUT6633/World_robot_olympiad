# v4.8 — Multiple Pillar Tracking

## What I Tried

Once pillar detection and distance estimation are working, we need to track multiple pillars across frames. The robot might see 2-3 pillars simultaneously, and as it moves, pillars enter and exit the field of view. I wrote `track_pillars.py` to maintain a persistent list of tracked pillars.

The first approach was simple proximity matching:
1. For each detected pillar in the current frame, find the nearest tracked pillar from the previous frame.
2. If the distance (in pixel space) is below a threshold, associate them.
3. If no match, start a new track.
4. If a track hasn't been updated in N frames, delete it.

```python
def match(self, detections):
    for d in detections:
        distances = [np.linalg.norm(d.center - t.center) for t in self.tracks]
        best = np.argmin(distances)
        if distances[best] < MATCH_THRESHOLD:
            t.update(d)
        else:
            self.tracks.append(Track(d))
```

## The Error — Pillars Disappear Between Frames

During a sharp turn, all tracked pillars disappeared:

```
[TRACK] [23.102] Track 0: last seen 0 frames ago, center=(320, 200)
[TRACK] [23.118] Track 0: last seen 1 frames ago, no match
[TRACK] [23.134] Track 0: last seen 2 frames ago, no match
[TRACK] [23.150] Track 0: PURGED (3 frames without update)
```

The robot was turning at about 90°/s. In the 16 ms between frames, the pillars shifted by 50-100 pixels due to the rotation. The match threshold was 40 pixels, so no detection matched the expected position.

Even after the turn completed and the pillars were back in view, the tracks were already purged. The robot had to re-detect pillars from scratch, causing a momentary "I see 0 pillars" panic in the navigation logic.

## What I Changed

I added a **Kalman filter** for each track that predicts the pillar's position in the next frame based on velocity.

The state vector: `[x, y, vx, vy]` (pixel position and velocity).

Prediction step:
```python
def predict(self):
    self.x = self.x + self.vx * self.dt
    self.y = self.y + self.vy * self.dt
    self.P = self.A @ self.P @ self.A.T + self.Q
```

Update step with new detection:
```python
def update(self, z):
    K = self.P @ self.H.T @ inv(self.H @ self.P @ self.H.T + self.R)
    self.x = self.x + K @ (z - self.H @ self.x)
    self.P = (np.eye(4) - K @ self.H) @ self.P
```

The Kalman filter's prediction means even if a pillar is not detected for 1-2 frames (due to motion blur, occlusion during turn, or colour threshold miss), the track continues at the predicted position. When the pillar reappears, it matches immediately.

I also increased the match threshold to 80 pixels when using Kalman prediction, since the prediction error is accounted for.

## Alternatives Considered

- **Optical flow for inter-frame tracking**: Compute dense optical flow (Farneback) between frames and use it to warp tracked positions. Too slow (~200 ms per frame at 640×480). The Kalman filter is essentially a constant-velocity model that costs ~0.01 ms per track.
- **SORT (Simple Online and Realtime Tracking)**: The standard multiple-object tracking algorithm. It uses Kalman filter + Hungarian algorithm. We don't need Hungarian matching for 2-3 objects — brute force is fine.
- **Feature matching (ORB)**: Extract ORB features from each pillar's region and match across frames. This could differentiate between two identical-looking red pillars. Overkill for now — they're all identical anyway.

## Still Broken

- **Track ID swap**: When two pillars cross in the camera's field of view (one passes behind another), the proximity matching may swap their track IDs. This is a fundamental limitation of position-only tracking. Deep SORT uses appearance features to avoid this, but all pillars look identical.
- **Initial velocity unknown**: When a new track is created, the velocity is initialised to zero. The Kalman filter takes ~5 frames to converge to the correct velocity. During this time, the prediction is poor. I could seed the velocity from the robot's own motion (odometry), which I might add in v5.x.

## Lesson Learned

Frame-by-frame matching without motion prediction is fragile during ego-motion. A simple constant-velocity Kalman filter adds enormous robustness for almost zero computational cost. It's worth implementing early rather than as an afterthought.
