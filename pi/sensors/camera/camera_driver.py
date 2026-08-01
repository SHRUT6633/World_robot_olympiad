# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/camera/camera_driver.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Raspberry Pi Camera Module driver
# =============================================================================

import cv2
import numpy as np
from ..base import SensorBase
from ...system.logger import log


class PiCamera(SensorBase):
    """
    Wrapper around OpenCV's VideoCapture for the Raspberry Pi Camera Module.

    Physical meaning:
      The camera captures visible-light frames from the CSI or USB camera
      interface. Each frame is a 2D array of RGB pixels. The robot uses
      these frames for:
        - Line / lane detection (field element recognition)
        - Object / obstacle classification (e.g. WRO game pieces)
        - Visual odometry / AprilTag localisation

    Video4Linux2 (V4L2) backend:
      We explicitly use cv2.CAP_V4L2 on Linux for lower latency and direct
      MMAP buffer access, bypassing OpenCV's internal FFMPEG decode.

    Configuration parameters:
      device : int
        /dev/video<device> device index. 0 = first camera. On RPi with
        the official camera module in legacy mode this is typically 0.
      width, height : int
        Requested frame resolution. Lower resolution = higher FPS but
        less detail. 640x480 is a good balance for line detection.
      fps : int
        Requested frames per second. The sensor may not achieve this if
        exposure time limits the frame rate (low light = longer exposure).
        60 fps is aggressive — 30 fps is more reliable.

    Buffersize:
      cv2.CAP_PROP_BUFFERSIZE = 2 limits the driver's internal queue to
      2 frames. This reduces latency: if the buffer grows large, the
      control loop sees older frames and reacts slowly.
    """

    def __init__(self, device=0, width=640, height=480, fps=60):
        # SensorBase sets self.name = "PiCamera".
        super().__init__("PiCamera")
        # V4L2 device number (/dev/video0 by default).
        self.device = device
        # Requested frame width in pixels.
        self.width = width
        # Requested frame height in pixels.
        self.height = height
        # Requested frames per second.
        self.fps = fps
        # cv2.VideoCapture object; None until init() succeeds.
        self._cap = None
        # Most recently captured frame (BGR numpy array), used by the
        # .frame property for read-after-write access patterns.
        self._frame = None
        # Total number of frames captured since init(). Useful for
        # diagnostics, FPS computation, and triggering periodic actions.
        self._frame_count = 0

    def init(self):
        """
        Open the camera device and configure resolution, frame rate, and
        buffer size.

        If the camera does not support the exact width/height/fps requested,
        V4L2 will silently choose the closest supported mode. Always check
        actual values with cap.get(PROP_*) if precise control is needed.
        """
        # Open /dev/video<device> with the V4L2 backend for low latency.
        # Open /dev/video<device> with V4L2 backend for lower latency than FFMPEG.
        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Camera /dev/video{self.device} could not be opened "
                f"(check CSI ribbon cable / USB connection and that the "
                f"camera is enabled: sudo raspi-config → Interface Options)")
        # Request resolution and framerate; V4L2 may silently round to a
        # supported mode if the exact values are not available.
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Minimise internal driver buffer to reduce latency — a large queue
        # would serve stale frames to the control loop.
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        # Log the ACTUAL mode the driver negotiated, not the requested one,
        # so the operator can spot a mismatch (e.g. 640x480 requested but
        # only 160x120 delivered).
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        f = int(self._cap.get(cv2.CAP_PROP_FPS))
        log.info(f"Camera: {w}x{h} @ {f}fps (/dev/video{self.device})")
        if w != self.width or h != self.height or f != self.fps:
            log.warn(f"Camera mode mismatch: requested "
                     f"{self.width}x{self.height}@{self.fps}fps, "
                     f"got {w}x{h}@{f}fps")

    def read_raw(self):
        """
        Grab a single frame from the camera.

        Returns:
          A 3D numpy array of shape (H, W, 3) in BGR order (OpenCV default),
          or None if the frame could not be captured (e.g. camera
          disconnected, or end of stream).

        Each pixel is an unsigned 8-bit integer (0–255). The frame is
        stored in self._frame so downstream code can access it without
        re-reading the sensor.
        """
        # Grab the next frame from V4L2 buffer; ret=False signals end of
        # stream or camera disconnection (e.g. loose CSI ribbon cable).
        ret, frame = self._cap.read()
        if not ret:
            return None
        # Cache the frame so .property can re-read without a new bus transaction.
        self._frame = frame
        self._frame_count += 1
        return frame

    @property
    def frame(self):
        """
        Return the most recently captured frame without issuing a new read.

        This is useful when the pipeline needs to access the same frame
        multiple times (e.g. for both line detection AND object detection)
        without consuming additional camera bandwidth.
        """
        return self._frame

    def close(self):
        """
        Release the camera device.

        This frees the V4L2 buffer and makes the camera available for
        other processes (e.g. libcamera, raspistill in another session).
        """
        if self._cap:
            self._cap.release()
