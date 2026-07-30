import cv2
import numpy as np
from ...system.logger import log


class CameraPipeline:
    """
    End-to-end image processing pipeline for the robot's camera.

    The pipeline transforms a raw camera frame into a clean, enhanced
    image ready for computer vision algorithms (line detection, object
    recognition, AprilTag decoding).

    Processing steps (in order):
      1. Undistort              — remove lens distortion using calibration data.
      2. Perspective transform  — warp to a bird's-eye view (top-down).
      3. Auto exposure          — brightness normalisation via gain.
      4. White balance          — remove colour cast from lighting.
      5. Gamma correction       — adjust mid-tone contrast.
      6. CLAHE                  — adaptive histogram equalisation for local contrast.
      7. Motion blur compensate — optional box blur / denoise.

    Each step can be enabled/disabled by passing the appropriate arguments
    to process().

    How the robot uses this:
      After this pipeline, the image is passed to the vision modules:
      - Line detection (Canny + Hough) for lane-keeping.
      - Colour thresholding (HSV range) for WRO game element detection.
      - AprilTag detector for localisation on the field.
    """

    def __init__(self):
        """
        Initialise the CLAHE (Contrast Limited Adaptive Histogram
        Equalisation) equaliser.

        clipLimit=2.0:
          Limits contrast amplification to 2×. Higher values increase
          noise amplification. 2.0 is a good balance.

        tileGridSize=(8, 8):
          The image is divided into 8×8 pixel tiles. Each tile is
          histogram-equalised independently. Smaller tiles adapt to
          local lighting better but can introduce block artefacts.
        """
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def undistort(self, img, mtx, dist):
        """
        Remove lens distortion using camera matrix and distortion coeffs.

        If mtx is None (calibration not loaded), the image is returned
        unchanged. This allows the pipeline to run without calibration
        data during development.
        """
        if mtx is None:
            return img
        return cv2.undistort(img, mtx, dist)

    def perspective_transform(self, img, src_pts, dst_pts, size):
        """
        Apply a bird's-eye perspective warp.

        src_pts: 4 source points in the original image (e.g. the four
                 corners of the field / lane segment).
        dst_pts: 4 destination points in the output (e.g. a rectangle).
        size:    (width, height) of the output image.

        Uses cv2.getPerspectiveTransform() to compute the 3×3 homography
        matrix H, then cv2.warpPerspective() to apply it.

        Bird's-eye view makes distance measurement and line following
        much easier because the floor geometry is approximately Euclidean.
        """
        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        return cv2.warpPerspective(img, H, size)

    def auto_exposure(self, img, target_brightness=128):
        """
        Adjust image brightness via digital gain so the average pixel
        value matches target_brightness (default 128 = mid-grey).

        gain = target / mean_luminance

        If the image is too dark (mean < target), gain > 1 brightens it.
        If too bright (mean > target), gain < 1 darkens it.

        np.clip(0, 255) prevents overflow. This is a crude digital
        auto-exposure — the real hardware exposure should also be adjusted
        via the camera driver's V4L2 controls.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mean = np.mean(gray)
        gain = target_brightness / (mean + 1e-6)
        return np.clip(img * gain, 0, 255).astype(np.uint8)

    def white_balance(self, img):
        """
        Simple white-balance in LAB colour space.

        The LAB colour space separates luminance (L) from colour channels
        (a = green–magenta, b = blue–yellow). By subtracting the mean
        of channels a and b weighted by luminance, we remove global
        colour casts caused by different lighting (e.g. indoor fluorescents
        vs. sunlight).

        The scaling factor 1.1 controls the correction strength.
        Higher values = more aggressive white balance.
        """
        result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        avg_a = np.mean(result[:, :, 1])
        avg_b = np.mean(result[:, :, 2])
        # Shift a and b channels toward neutral, weighted by L channel.
        result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
        return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)

    def gamma_correction(self, img, gamma=1.2):
        """
        Apply gamma correction for mid-tone contrast adjustment.

        output = 255 * (input / 255) ^ (1/gamma)

        gamma > 1 : brightens mid-tones (image appears lighter).
        gamma < 1 : darkens mid-tones (image appears darker).
        gamma = 1 : identity (no change).

        Gamma correction compensates for the non-linear luminance response
        of typical monitors and cameras. A gamma of ~1.2–1.5 often makes
        field elements more distinguishable.
        """
        inv = 1.0 / gamma
        # Pre-compute a lookup table for all 256 possible pixel values.
        table = np.array([(i / 255.0) ** inv * 255 for i in range(256)]).astype("uint8")
        # cv2.LUT applies the table to every pixel efficiently (vectorised).
        return cv2.LUT(img, table)

    def apply_clahe(self, img):
        """
        Apply Contrast Limited Adaptive Histogram Equalisation to the
        Luminance channel (L in LAB).

        CLAHE enhances local contrast without amplifying noise in uniform
        areas (unlike global histogram equalisation). This is critical for:
          - Detecting lines on a field with varying illumination (shadows).
          - Reading AprilTags at different distances.

        Only the L channel is equalised; colour channels a and b are
        preserved to avoid colour shift.
        """
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        # Apply CLAHE to the L channel only.
        lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def motion_blur_compensate(self, img, kernel_size=3):
        """
        Apply a simple box blur / averaging filter to reduce noise.

        This is NOT true motion de-blur — it is a smoothing operation.
        A 3×3 kernel averages each pixel with its 8 neighbours:

            K = (1/9) * [[1, 1, 1],
                         [1, 1, 1],
                         [1, 1, 1]]

        Larger kernel_size = more blur = more noise reduction but also
        loss of fine detail. kernel_size must be odd.
        Setting kernel_size < 2 disables filtering.
        """
        if kernel_size < 2:
            return img
        kernel = np.ones((kernel_size, kernel_size), np.float32) / (kernel_size * kernel_size)
        return cv2.filter2D(img, -1, kernel)

    def process(self, img, mtx=None, dist=None, src_pts=None, dst_pts=None, warp_size=None):
        """
        Run the full image processing pipeline.

        Parameters:
          img       : raw BGR frame from PiCamera.read_raw().
          mtx, dist : camera calibration data (from CameraCalibration).
          src_pts   : 4 source points for perspective warp (or None to skip).
          dst_pts   : 4 destination points for perspective warp.
          warp_size : (w, h) output size for warp.

        Returns the processed image, or None if input was None.
        """
        if img is None:
            return None
        # Step 1: Remove lens distortion.
        img = self.undistort(img, mtx, dist)
        # Step 2: Warp to bird's-eye view if source points are provided.
        if src_pts is not None and dst_pts is not None and warp_size is not None:
            img = self.perspective_transform(img, src_pts, dst_pts, warp_size)
        # Step 3: Normalise brightness.
        img = self.auto_exposure(img)
        # Step 4: Remove colour cast.
        img = self.white_balance(img)
        # Step 5: Adjust mid-tone contrast.
        img = self.gamma_correction(img)
        # Step 6: Enhance local contrast.
        img = self.apply_clahe(img)
        return img
