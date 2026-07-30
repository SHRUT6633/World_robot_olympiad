import numpy as np


class ComplementaryFilter:
    def __init__(self, alpha=0.98, dt=0.01):
        self.alpha = alpha
        self.dt = dt
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

    def update(self, accel, gyro, mag_heading=None):
        accel_pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        accel_roll = np.arctan2(accel[1], accel[2])

        self.pitch = self.alpha * (self.pitch + gyro[0] * self.dt) + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * (self.roll + gyro[1] * self.dt) + (1 - self.alpha) * accel_roll

        if mag_heading is not None:
            gyro_yaw = self.yaw + gyro[2] * self.dt
            self.yaw = self.alpha * gyro_yaw + (1 - self.alpha) * mag_heading
        else:
            self.yaw += gyro[2] * self.dt

        return self.pitch, self.roll, self.yaw
