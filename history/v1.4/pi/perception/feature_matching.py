# =============================================================================
# feature_matching.py — ORB feature extraction + brute-force matching
# =============================================================================
# Provides a reusable ORB (Oriented FAST and Rotated BRIEF) feature
# extractor and a brute-force Hamming-distance matcher.
#
# This is the backbone of:
#   - Visual odometry (matching features frame-to-frame)
#   - Loop closure detection (matching current view against a map)
#   - Landmark recognition (see also landmark_detection.py)
# =============================================================================

import cv2                                  # OpenCV
import numpy as np                          # (used indirectly via OpenCV)


class FeatureMatcher:
    """
    ORB feature extractor + BFMatcher (Hamming, cross-checked).

    Parameters
    ----------
    nfeatures : int
        Number of ORB features to extract per image.
        More features → more matches possible but slower computation.
        Typical values: 500 (good balance), 1000 (for rich scenes), 200+ (low power).

    Connecting to the system
    ------------------------
    - extract() is called once per frame to produce keypoints + descriptors.
    - match() is called between consecutive frames (or against a map) to
      obtain putative correspondences.
    """

    def __init__(self, nfeatures=500):
        # ORB detector/descriptor — scale-pyramid-based, rotation-invariant
        self.orb = cv2.ORB_create(nfeatures=nfeatures)

        # Brute-force matcher using Hamming distance (correct for ORB).
        # crossCheck=True ensures symmetry: only returns (i,j) if i's best
        # match is j AND j's best match is i — drastically reduces outliers.
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # ------------------------------------------------------------------
    # extract — Keypoints & descriptors for one image
    # ------------------------------------------------------------------
    def extract(self, img):
        """
        Run ORB detect-and-compute on *img*.

        Returns
        -------
        tuple (kp, des)
            kp  : list of cv2.KeyPoint
            des : np.ndarray of shape (N, 32)  (ORB produces 32-byte descriptors)
            Returns (None, None) if no features are found.

        Note
        ----
        *img* should be grayscale for best performance; if BGR is passed,
        ORB works on the first channel only.
        """
        kp, des = self.orb.detectAndCompute(img, None)
        return kp, des

    # ------------------------------------------------------------------
    # match — Correspondences between two descriptor sets
    # ------------------------------------------------------------------
    def match(self, des1, des2):
        """
        Brute-force match *des1* → *des2* using Hamming distance.

        Returns
        -------
        list of cv2.DMatch
            Sorted by distance (ascending, i.e. best matches first).
            Empty list if either descriptor set is None.

        What if you disable crossCheck?
        - Without crossCheck, you get ~2× more matches but many are wrong.
        - You would then need a ratio test (Lowe's) to prune them.
        """
        if des1 is None or des2 is None:
            return []                         # No descriptors → no matches

        matches = self.bf.match(des1, des2)   # Cross-checked brute-force match

        # Sort so the best (smallest distance) matches come first.
        # Downstream code often takes only the top-k or applies a threshold.
        return sorted(matches, key=lambda x: x.distance)
