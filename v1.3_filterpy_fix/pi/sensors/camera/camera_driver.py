import cv2
import numpy as np
from ..base import SensorBase
from ...system.logger import log


class PiCamera(SensorBase):
    def __init__(self, device=0, width=640, height=480, fps=60):
        super().__init__("PiCamera")
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self._cap = None
        self._frame = None
        self._frame_count = 0

    def init(self):
        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        log.info(f"Camera: {self.width}x{self.height} @ {self.fps}fps")

    def read_raw(self):
        ret, frame = self._cap.read()
        if not ret:
            return None
        self._frame = frame
        self._frame_count += 1
        return frame

    @property
    def frame(self):
        return self._frame

    def close(self):
        if self._cap:
            self._cap.release()
