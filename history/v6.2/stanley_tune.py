import numpy as np
import math


def run_standalone_test():
    controller = StanleyController()

    path = np.array([
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 2.0],
        [0.0, 2.0],
        [0.0, 0.0],
    ])

    dense = np.linspace(0, 1, len(path))
    dense_x = np.interp(np.linspace(0, 1, 200), dense, path[:, 0])
    dense_y = np.interp(np.linspace(0, 1, 200), dense, path[:, 1])
    dense_path = np.column_stack((dense_x, dense_y))

    test_speeds = [0.2, 0.5, 1.0, 1.5]
    k_values = [0.3, 0.5, 0.8, 1.0, 1.5]

    for v in test_speeds:
        print(f"\n--- Speed {v} m/s ---")
        for k in k_values:
            controller.k = k
            cte_sum = 0.0
            n = 0

            x, y, heading = 0.1, 0.1, 0.0

            for _ in range(100):
                steer = controller.compute(x, y, heading, dense_path, v)
                heading += steer * 0.01
                x += v * math.cos(heading) * 0.01
                y += v * math.sin(heading) * 0.01

                nearest = controller.nearest_point(x, y, dense_path)
                cte = -np.sin(heading) * (nearest[0][0] - x) + np.cos(heading) * (nearest[0][1] - y)
                cte_sum += abs(cte)
                n += 1

            avg_cte = cte_sum / n
            print(f"  k={k:.1f}  avg_CTE={avg_cte:.4f} m")
