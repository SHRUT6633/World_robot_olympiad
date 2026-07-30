import numpy as np
import math


def perfect_circle_path(radius, center, num_pts=100):
    angles = np.linspace(0, np.pi / 2, num_pts)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return np.column_stack((x, y))


def run_corner_test():
    controller = FeedforwardSteering(wheelbase=0.26, max_ff_ratio=0.5)

    path = perfect_circle_path(0.5, (0, 0), 100)
    x, y, heading = 0.5, 0.0, 0.0

    v = 0.8
    dt = 0.01
    total_error = 0.0
    n = 0

    for t in range(500):
        dists = np.hypot(path[:, 0] - x, path[:, 1] - y)
        idx = int(np.argmin(dists))

        dx = path[idx, 0] - x
        dy = path[idx, 1] - y
        cte = -np.sin(heading) * dx + np.cos(heading) * dy

        if idx + 1 < len(path):
            th = np.arctan2(path[idx + 1, 1] - path[idx, 1], path[idx + 1, 0] - path[idx, 0])
        else:
            th = heading
        he = np.arctan2(np.sin(th - heading), np.cos(th - heading))
        feedback = he + np.arctan2(1.0 * cte, 1.0 + v)

        curvature = controller.compute_curvature(path, idx)
        steer = controller.compute(feedback, curvature)

        heading += steer * dt
        x += v * math.cos(heading) * dt
        y += v * math.sin(heading) * dt

        cte_at = -np.sin(heading) * (path[idx, 0] - x) + np.cos(heading) * (path[idx, 1] - y)
        total_error += abs(cte_at)
        n += 1

    print(f"Average cross-track error: {total_error / n:.4f} m")
    print(f"Expected: < 0.05 m for well-tuned controller")
