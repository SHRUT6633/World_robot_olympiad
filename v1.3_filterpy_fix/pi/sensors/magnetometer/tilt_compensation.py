import numpy as np


class TiltCompensation:
    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0

    def update_orientation(self, accel):
        norm = np.linalg.norm(accel) + 1e-6
        self.pitch = np.arcsin(-accel[0] / norm)
        self.roll = np.arctan2(accel[1], accel[2])

    def compensate(self, mag):
        x = mag[0] * np.cos(self.pitch) + mag[2] * np.sin(self.pitch)
        y = (mag[0] * np.sin(self.roll) * np.sin(self.pitch) +
             mag[1] * np.cos(self.roll) -
             mag[2] * np.sin(self.roll) * np.cos(self.pitch))
        return np.array([x, y, mag[2]])
