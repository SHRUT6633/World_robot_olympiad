import numpy as np


class CrossVerify:
    def __init__(self, tolerance: float = 0.10, extreme_threshold: float = 0.25):
        self.tolerance = tolerance
        self.extreme_threshold = extreme_threshold
        self.x_offset = 0.05
        self.yaw_offset = np.radians(0.5)
        self.tof_health = 1.0
        self.camera_health = 1.0

    def tof_to_camera(self, tof_dist: float, robot_heading: float,
                      wall_angle: float) -> float:
        theta = robot_heading + self.yaw_offset
        dx = self.x_offset * np.cos(theta)
        cam_dist = np.sqrt(
            dx**2 + tof_dist**2 + 2 * dx * tof_dist * np.cos(wall_angle)
        )
        return cam_dist

    def verify(self, tof_dist: float, camera_dist: float,
               robot_heading: float, wall_angle: float) -> dict:
        tof_in_cam = self.tof_to_camera(tof_dist, robot_heading, wall_angle)
        diff = abs(tof_in_cam - camera_dist)

        result = {
            "tof_corrected": tof_in_cam,
            "camera_raw": camera_dist,
            "diff": diff,
            "accept_tof": True,
            "accept_camera": True,
            "fused_distance": None,
        }

        if diff > self.extreme_threshold:
            result["accept_tof"] = False
            result["accept_camera"] = False
            self.tof_health *= 0.9
            self.camera_health *= 0.9
            result["fused_distance"] = None
        elif diff > self.tolerance:
            if self.tof_health > self.camera_health:
                result["accept_tof"] = True
                result["accept_camera"] = False
                self.camera_health *= 0.95
                result["fused_distance"] = tof_in_cam
            else:
                result["accept_tof"] = False
                result["accept_camera"] = True
                self.tof_health *= 0.95
                result["fused_distance"] = camera_dist
        else:
            w_tof = self.tof_health / (self.tof_health + self.camera_health)
            w_cam = 1.0 - w_tof
            result["fused_distance"] = w_tof * tof_in_cam + w_cam * camera_dist

        return result
