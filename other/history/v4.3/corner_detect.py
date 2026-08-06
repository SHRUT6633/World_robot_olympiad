import math


CORNER_THRESHOLD_DEG = 45.0
EXIT_THRESHOLD_DEG = 85.0
GYRO_SCALE = 1.0 / 131.0


class CornerDetect:
    def __init__(self):
        self.integrated_yaw = 0.0
        self.entry_yaw = 0.0
        self.waiting_for_corner = True
        self.waiting_for_exit = False
        self.corner_count = 0
        self.last_time = None

    def update(self, gyro_z_dps, dt):
        self.integrated_yaw += gyro_z_dps * dt

        if self.waiting_for_corner:
            if abs(self.integrated_yaw) > CORNER_THRESHOLD_DEG:
                self.entry_yaw = self.integrated_yaw
                self.waiting_for_corner = False
                self.waiting_for_exit = True
                return {"state": "entry", "yaw": self.integrated_yaw}

        if self.waiting_for_exit:
            delta = abs(self.integrated_yaw - self.entry_yaw)
            if delta > EXIT_THRESHOLD_DEG:
                self.corner_count += 1
                self.waiting_for_exit = False
                self.waiting_for_corner = True
                self.integrated_yaw = self.integrated_yaw - self.entry_yaw
                return {"state": "exit", "yaw": self.integrated_yaw, "count": self.corner_count}

        return {"state": "straight", "yaw": self.integrated_yaw}
