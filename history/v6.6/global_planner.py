import math


class GlobalPlanner:
    def __init__(self, center_offset=0.2):
        self.center_offset = center_offset
        self.waypoints = []

    def plan_rectangle(self, width, height):
        cx = self.center_offset
        cy = self.center_offset
        w = width - cx
        h = height - cy

        self.waypoints = [
            (cx, cy),
            (w, cy),
            (w, h),
            (cx, h),
            (cx, cy),
        ]
        return self.waypoints

    def interpolate(self, spacing=0.1):
        if not self.waypoints:
            return self.waypoints

        new_pts = []
        for i in range(len(self.waypoints) - 1):
            p0 = self.waypoints[i]
            p1 = self.waypoints[i + 1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            dist = math.hypot(dx, dy)
            n = max(2, int(dist / spacing))
            for j in range(n):
                t = j / n
                new_pts.append((p0[0] + t * dx, p0[1] + t * dy))
        new_pts.append(self.waypoints[-1])
        self.waypoints = new_pts
        return self.waypoints

    def get_target(self, index):
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None

    def reverse_direction(self):
        self.waypoints.reverse()
        return self.waypoints
