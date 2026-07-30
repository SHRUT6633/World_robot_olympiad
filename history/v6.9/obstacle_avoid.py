import numpy as np


class DynamicObstacleAvoidance:
    def __init__(self, detection_radius=0.5, offset_dist=0.3):
        self.radius = detection_radius
        self.offset = offset_dist
        self.paths = {}
        self.active_path = "center"

    def _offset_path(self, path, offset):
        result = np.copy(path)
        for i in range(1, len(path) - 1):
            dx = path[i + 1, 0] - path[i - 1, 0]
            dy = path[i + 1, 1] - path[i - 1, 1]
            norm = np.hypot(dx, dy) + 1e-6
            result[i, 0] += -dy / norm * offset
            result[i, 1] += dx / norm * offset
        return result

    def _brake_path(self, path):
        result = np.copy(path)
        return result

    def precompute(self, nominal_path):
        self.paths = {
            "center": np.array(nominal_path),
            "left": self._offset_path(np.array(nominal_path), self.offset),
            "right": self._offset_path(np.array(nominal_path), -self.offset),
            "brake": self._brake_path(np.array(nominal_path)),
        }
        self.active_path = "center"

    def _side_of_path(self, obstacle, robot_pose):
        rx, ry, rtheta = robot_pose
        ox, oy = obstacle[:2]
        dx = ox - rx
        dy = oy - ry
        side = -np.sin(rtheta) * dx + np.cos(rtheta) * dy
        return "left" if side > 0 else "right"

    def select_path(self, obstacles, robot_pose):
        for obs in obstacles:
            dist = np.hypot(obs[0] - robot_pose[0], obs[1] - robot_pose[1])
            if dist < self.radius:
                side = self._side_of_path(obs, robot_pose)
                chosen = "right" if side == "left" else "left"
                self.active_path = chosen
                return self.paths[chosen]

        self.active_path = "center"
        return self.paths["center"]

    def avoid(self, current_waypoint, robot_pose, obstacles):
        if not obstacles:
            return current_waypoint

        nearest = min(obstacles, key=lambda o: np.linalg.norm(np.array(o) - np.array(robot_pose[:2])))
        dist = np.linalg.norm(np.array(nearest) - np.array(robot_pose[:2]))

        if dist < self.radius:
            dx = robot_pose[0] - nearest[0]
            dy = robot_pose[1] - nearest[1]
            norm = np.linalg.norm([dx, dy]) + 1e-6
            avoidance = np.array([dx / norm, dy / norm]) * 0.3
            return (current_waypoint[0] + avoidance[0], current_waypoint[1] + avoidance[1])

        return current_waypoint
