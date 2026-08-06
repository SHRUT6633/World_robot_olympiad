from enum import Enum


class ImuCorrectionMode(Enum):
    FULL = "full"
    YAW_ONLY = "yaw_only"
    GYRO_ONLY = "gyro_only"


class ImuModeController:
    def __init__(self):
        self._mode = ImuCorrectionMode.FULL

    def set_mode(self, mode: ImuCorrectionMode):
        self._mode = mode

    def get_mode(self) -> ImuCorrectionMode:
        return self._mode

    def apply_to_filter(self, mahony_filter):
        if self._mode == ImuCorrectionMode.FULL:
            mahony_filter.kp = 2.0
            mahony_filter.ki = 0.1
        elif self._mode == ImuCorrectionMode.YAW_ONLY:
            mahony_filter.kp = 2.0
            mahony_filter.ki = 0.1
        elif self._mode == ImuCorrectionMode.GYRO_ONLY:
            mahony_filter.kp = 0.0
            mahony_filter.ki = 0.0
