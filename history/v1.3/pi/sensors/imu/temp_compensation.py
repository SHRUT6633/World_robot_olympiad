import numpy as np
from ...system.logger import log


class IMUTempCompensation:
    def __init__(self):
        self.ref_temp = 25.0
        self.accel_temp_coeff = np.array([0.002, 0.002, 0.002])
        self.gyro_temp_coeff = np.array([0.02, 0.02, 0.02])

    def read_temp(self):
        return 25.0

    def compensate(self, accel, gyro, temp=None):
        if temp is None:
            temp = self.read_temp()
        dt = temp - self.ref_temp
        accel_comp = accel - self.accel_temp_coeff * dt
        gyro_comp = gyro - self.gyro_temp_coeff * dt
        return accel_comp, gyro_comp
