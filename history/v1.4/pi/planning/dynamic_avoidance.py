import numpy as np


class DynamicObstacleAvoidance:
    # ------------------------------------------------------------------
    # 1) Constructor: sets the detection radius in metres.
    #
    #    detection_radius=0.5
    #      – The robot will begin to steer away when an obstacle is
    #        within 0.5 m of its current pose.
    #      – Making this larger makes the robot "nervous" (avoids early);
    #        making it smaller makes the robot "brave" (avoids late).
    # ------------------------------------------------------------------
    def __init__(self, detection_radius=0.5):
        self.radius = detection_radius

    # ------------------------------------------------------------------
    # 2) avoid(current_waypoint, robot_pose, obstacles) -> (x, y) tuple
    #
    #    This is the core avoidance logic.  It is called during every
    #    planning tick (e.g. 20 Hz) to possibly deflect the nominal
    #    waypoint away from nearby obstacles.
    #
    #    Parameters:
    #      current_waypoint : (x, y)  – the target point the robot would
    #                                    drive toward without obstacles.
    #      robot_pose       : (x, y, theta) – the robot's current pose.
    #      obstacles        : list of (x, y) – positions of detected
    #                         obstacles (usually from ObjectDetector
    #                         or depth estimation).
    #
    #    Algorithm:
    #      1) If no obstacles, return the waypoint unchanged.
    #      2) Find the *nearest* obstacle (Euclidean distance to robot).
    #      3) If the distance to that obstacle < self.radius, compute
    #         a unit vector pointing *away* from the obstacle:
    #
    #             avoidance = (away_x, away_y) * 0.3
    #
    #         The 0.3 multiplier is the "push strength".  Increasing it
    #         makes the robot dodge harder; decreasing it makes the
    #         correction more subtle.
    #      4) The adjusted waypoint = original + avoidance vector.
    #         This effectively "shoves" the target in the safe direction.
    #
    #    Important detail:
    #      - norm is incremented by 1e-6 to avoid division by zero when
    #        the robot is exactly on top of the obstacle.
    #
    #    What if detection_radius is 0?
    #      – The robot never avoids (unless dist < 0, impossible).
    #
    #    Connection to the system:
    #      - This method is called from the main planning loop after
    #        object_detection.py has identified obstacles.
    #      - The modified waypoint then flows to WaypointOptimizer and
    #        finally to the trajectory generators (BezierCurve,
    #        CurvatureOptimizer, etc.).
    # ------------------------------------------------------------------
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
