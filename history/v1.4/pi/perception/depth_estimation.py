import numpy as np


class DepthEstimator:
    # ------------------------------------------------------------------
    # 1) Constructor: sets up stereo depth estimation parameters.
    #    baseline_m  – distance between the two stereo cameras (in metres).
    #                   A wider baseline gives greater depth sensitivity but
    #                   also makes correspondence harder.
    #    focal_px    – focal length of the cameras in pixels.  This is
    #                   typically extracted from the camera calibration
    #                   matrix (fx).  A longer focal length magnifies depth
    #                   errors for distant objects.
    # ------------------------------------------------------------------
    def __init__(self, baseline_m=0.1, focal_px=300):
        self.baseline = baseline_m
        self.focal = focal_px

    # ------------------------------------------------------------------
    # 2) from_disparity(disparity) -> depth map (numpy array)
    #
    #    Converts a disparity map (horizontal pixel shift between left &
    #    right images) into a metric depth map using the classic formula:
    #
    #        depth = (focal * baseline) / disparity
    #
    #    "disparity" is a 2-D array where each pixel holds the shift value
    #    (>= 0).  Pixels with disparity == 0 are clamped to zero depth.
    #
    #    mask – boolean array that is True where disparity > 0.
    #           We only compute depth for those valid pixels.
    #    depth – output array, same shape as disparity, float64.
    #            For valid pixels: depth = (focal * baseline) / disparity.
    #            Invalid pixels stay at 0.0.
    #
    #    What happens if you change baseline / focal?
    #      - A larger product (focal * baseline) makes every depth
    #        estimate larger (objects appear farther away).
    #      - A smaller product makes depth estimates smaller (objects
    #        appear closer).  This is a linear scaling.
    #
    #    What happens if you skip the mask?
    #      - Division by zero would produce +inf values, poisoning
    #        downstream navaids or filters.
    # ------------------------------------------------------------------
    def from_disparity(self, disparity):
        mask = disparity > 0
        depth = np.zeros_like(disparity, dtype=float)
        depth[mask] = (self.focal * self.baseline) / disparity[mask]
        return depth
