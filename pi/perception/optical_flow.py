# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/optical_flow.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Lucas-Kanade optical flow estimation
# =============================================================================

import cv2
import numpy as np


class OpticalFlow:
    # OpticalFlow computes the average pixel motion between consecutive frames
    # using the Lucas-Kanade sparse optical flow method (pyramidal implementation).
    # The resulting flow vector can be used to estimate the robot's lateral and
    # forward velocity — a form of visual odometry that is fast (CPU-cheap) but
    # noisy.  The output feeds into sensor fusion and can assist the control loop
    # with rapid response to movement changes.
    #
    # Why Lucas-Kanade instead of dense (Farneback) flow?
    #   - Sparse LK tracks only corner features (50 by default), which is much
    #     faster on a Raspberry Pi than computing dense flow for every pixel.
    #   - Dense flow would provide per-pixel motion but at 2–3× the CPU cost.
    #
    # Why not use VisualOdometry instead?
    #   - VisualOdometry (essential-matrix-based) is more accurate but requires
    #     at least 8 good matches and works best with forward motion.  Optical
    #     flow works even during pure rotation or sideways movement, so it serves
    #     as a complementary signal for the sensor fusion UKF.

    def __init__(self):
        # Stores the previous frame's grayscale image for the next computation.
        self._prev_gray = None

        # Lucas-Kanade pyramidal optical flow parameters:
        #   winSize=(15,15)   — search window at each pyramid level.  Larger
        #                       windows catch larger displacements but are slower.
        #   maxLevel=2        — number of pyramid levels (3 total: full + 2
        #                       down-sampled).  More levels help track larger
        #                       inter-frame motions.
        #   criteria          — termination criteria: stop after 10 iterations
        #                       or when epsilon < 0.03.
        #
        # False-positive rejection:
        #   - OpenCV's calcOpticalFlowPyrLK returns a "status" flag per point (1 =
        #     tracked successfully, 0 = lost).  We only use points with status == 1.
        #   - Flow vectors from poorly tracked points (e.g. illumination change,
        #     occlusion) are automatically discarded by this status check.
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
        # img — current BGR camera frame.
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
        #   maxCorners=50       — max features to track (higher = more coverage,
        #                         slower).
        #   qualityLevel=0.01   — minimum quality as fraction of best eigenvalue.
        #                         Lower → more (weaker) corners.
        #   minDistance=5       — min pixel distance between corners.  Higher →
        #                         corners spread out more.
        #
        # Shi-Tomasi is used (not FAST/ORB) because the corner quality metric
        # (minimum eigenvalue) correlates well with trackability in LK.
        corners = cv2.goodFeaturesToTrack(self._prev_gray, 50, 0.01, 5)

        if corners is None:
            # No features to track in the previous frame (e.g. blank wall).
            self._prev_gray = gray
            return None

        # Track corners into the current frame using pyramidal LK optical flow.
        # Returns:
        #   next_pts — (N, 1, 2) array of tracked point positions.
        #   status   — (N, 1) byte array: 1 = success, 0 = failure.
        #   err      — (N, 1) error measure.
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, corners, None, **self._lk_params
        )

        self._prev_gray = gray

        # Only keep points where tracking succeeded (status == 1).
        good = status.ravel() == 1

        if not good.any():
            return None

        # Compute displacement vectors for successfully tracked points.
        flow = next_pts[good] - corners[good]

        # Return the average flow across all tracked features.
        # (dx, dy) represents the mean pixel motion from prev to current frame.
        return np.mean(flow, axis=0)
