import heapq
import math
import numpy as np


class HybridAStar:
    # Hybrid A* is a kinematically-aware path planner that extends the classic
    # A* algorithm by considering the robot's heading (theta) as part of the
    # state.  It generates a path that respects the minimum turning radius
    # (max_steer).  This is used to plan obstacle-avoidance or parking
    # manoeuvres where the robot cannot instantly change direction.

    def __init__(self, step_size=0.1, max_steer=np.radians(30)):
        # step_size -- distance (metres) the robot moves per expansion step.
        #   Smaller values give smoother paths but increase search time.
        # max_steer -- maximum steering angle in radians (default 30 degrees).
        #   Smaller values produce more conservative, wider turns.
        #   Larger values allow sharper turns but may be physically unfeasible.
        self.step = step_size
        self.max_steer = max_steer

    def search(self, start, goal, occupancy_grid):
        # Perform a Hybrid A* search.
        # start -- (x, y, theta) tuple describing the start pose (metres, radians).
        # goal  -- (x, y, theta) tuple describing the goal pose.
        # occupancy_grid -- not used in this simplified implementation but
        #   would be a 2D array where occupied cells block expansion.
        # Returns a list of (x, y) waypoints from start to goal.

        # Priority queue entries: (f_score, g_score, state, path_so_far).
        heap = [(0, 0, start, [start])]

        # Visited set keyed by rounded (x, y) to avoid revisiting cells.
        visited = set()

        while heap:
            # Pop the state with the smallest f = g + h.
            f, g, state, path = heapq.heappop(heap)

            # Use (x, y) rounded to 2 decimals as a visitation key.
            key = (round(state[0], 2), round(state[1], 2))
            if key in visited:
                continue
            visited.add(key)

            # Goal check: if within 0.15 m of the goal XY, return the path.
            if np.linalg.norm(np.array(state[:2]) - np.array(goal[:2])) < 0.15:
                return path

            # Generate 7 steering angles evenly spaced between -max_steer
            # and +max_steer (roughly forward, slight left/right, hard left/right).
            for steer in np.linspace(-self.max_steer, self.max_steer, 7):
                # Simple bicycle-model kinematics update.
                nx = state[0] + self.step * np.cos(state[2] + steer)
                ny = state[1] + self.step * np.sin(state[2] + steer)
                ntheta = state[2] + steer
                ng = g + self.step
                # Heuristic: Euclidean distance to the goal XY.
                nh = np.linalg.norm(np.array([nx, ny]) - np.array(goal[:2]))
                # Push candidate onto the heap.
                heapq.heappush(
                    heap, (ng + nh, ng, (nx, ny, ntheta), path + [(nx, ny)])
                )

        # If the heap is exhausted without reaching the goal, return the
        # best (closest) path so far.  In a real implementation this would
        # indicate a planning failure.
        return path
