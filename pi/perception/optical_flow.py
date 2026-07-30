import cv2
import numpy as np


class OpticalFlow:
    # OpticalFlow computes the average pixel motion between consecutive frames
    # using the Lucas-Kanade sparse optical flow method.  The resulting flow
    # vector can be used to estimate the robot's lateral and forward velocity
    # (a form of visual odometry that is fast but noisy).  The output feeds
    # into sensor fusion and can assist the control loop with rapid response
    # to movement changes.

    def __init__(self):
        # Stores the previous frame's grayscale image for the next computation.
        self._prev_gray = None

        # Lucas-Kanade parameters:
        #   winSize=(15,15)     -- size of the search window at each pyramid
        #                         level.  Larger windows can catch larger
        #                         displacements but are slower.
        #   maxLevel=2          -- number of pyramid levels (2 -> 3 levels
        #                         total: full + 2 down-sampled).  More levels
        #                         help track larger motions.
        #   criteria            -- termination criteria: stop after 10 iterations
        #                         or when epsilon < 0.03.
        self._lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                10,
                0.03,
            ),
        )

    def compute(self, img):
        # img -- current BGR camera frame.
        # Returns the mean (dx, dy) flow vector across all tracked corners,
        # or None if tracking is unavailable (first frame, no corners found,
        # or all points are lost).

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # First frame: save it and return None (no motion yet).
        if self._prev_gray is None:
            self._prev_gray = gray
            return None

        # Detect Shi-Tomasi corner features in the previous frame.
        # Parameters:
        #   maxCorners=50  -- maximum number of corners to track.
        #   qualityLevel=0.01  -- minimum quality (lower = more corners).
        #   minDistance=5  -- minimum Euclidean distance between corners.
        corners = cv2.goodFeaturesToTrack(self._prev_gray, 50, 0.01, 5)

        if corners is None:
            # No features to track in the previous frame.
            self._prev_gray = gray
            return None

        # Track corners into the current frame using pyramidal LK optical flow.
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, corners, None, **self._lk_params
        )

        self._prev_gray = gray

        # status == 1 means the feature was successfully tracked.
        good = status.ravel() == 1

        if not good.any():
            return None

        # Compute displacement vectors for successfully tracked points.
        flow = next_pts[good] - corners[good]

        # Return the average flow across all tracked features.
        return np.mean(flow, axis=0)
