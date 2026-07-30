import math
import numpy as np

RATE_THRESHOLD = math.radians(90.0)
GYRO_BIAS_SAMPLES = 500
K_ACCEL = 0.02
K_MAG = 0.01


class ComplementaryFull:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_bias = np.zeros(3)
        self._bias_buffer = []
        self._bias_collected = False

    def update(self, accel: np.ndarray, gyro: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple:
        gyro_unbiased = gyro - self.gyro_bias
        angular_rate = np.linalg.norm(gyro_unbiased)

        gyro_trust = 1.0
        if angular_rate > RATE_THRESHOLD:
            gyro_trust = RATE_THRESHOLD / angular_rate

        self.roll += (gyro_unbiased[0] + (1 - gyro_trust) * self._accel_roll_correction(accel, dt)) * dt
        self.pitch += (gyro_unbiased[1] + (1 - gyro_trust) * self._accel_pitch_correction(accel, dt)) * dt
        self.yaw += (gyro_unbiased[2] + (1 - gyro_trust) * self._mag_yaw_correction(mag, dt)) * dt

        return self.roll, self.pitch, self.yaw

    def _accel_roll_correction(self, accel: np.ndarray, dt: float) -> float:
        accel_roll = math.atan2(accel[1], accel[2])
        return K_ACCEL * (accel_roll - self.roll) / dt

    def _accel_pitch_correction(self, accel: np.ndarray, dt: float) -> float:
        accel_pitch = math.atan2(-accel[0],
                                 math.sqrt(accel[1]**2 + accel[2]**2))
        return K_ACCEL * (accel_pitch - self.pitch) / dt

    def _mag_yaw_correction(self, mag: np.ndarray, dt: float) -> float:
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        mx = mag[0] * cp + mag[1] * sr * sp + mag[2] * cr * sp
        my = mag[1] * cr - mag[2] * sr
        mag_yaw = math.atan2(-my, mx)
        return K_MAG * (mag_yaw - self.yaw) / dt

    def calibrate_gyro_bias(self, gyro_samples: list) -> None:
        self.gyro_bias = np.mean(gyro_samples, axis=0)
        self._bias_collected = True
