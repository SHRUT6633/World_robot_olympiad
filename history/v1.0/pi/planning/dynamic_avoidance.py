import numpy as np


class DynamicObstacleAvoidance:
    def __init__(self, detection_radius=0.5):
        self.radius = detection_radius

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
