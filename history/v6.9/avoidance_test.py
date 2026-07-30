import numpy as np
import time


def generate_nominal_path():
    pts = []
    for i in range(100):
        x = i * 0.03
        y = 1.0 + 0.2 * np.sin(x * 2.0)
        pts.append((x, y))
    return np.array(pts)


def test_precompute_timing():
    nominal = generate_nominal_path()
    avoid = DynamicObstacleAvoidance(detection_radius=0.5, offset_dist=0.3)

    t0 = time.perf_counter()
    avoid.precompute(nominal)
    t_pre = time.perf_counter() - t0

    print(f"Precompute time: {t_pre * 1000:.1f} ms")
    print(f"  Paths available: {list(avoid.paths.keys())}")
    print(f"  Center path: {len(avoid.paths['center'])} pts")
    print(f"  Left path:   {len(avoid.paths['left'])} pts")
    print(f"  Right path:  {len(avoid.paths['right'])} pts")

    if t_pre < 0.01:
        print("PASS: Precomputation under 10 ms")
    else:
        print(f"WARNING: Precomputation took {t_pre*1000:.1f} ms")


def test_switch_timing():
    nominal = generate_nominal_path()
    avoid = DynamicObstacleAvoidance()
    avoid.precompute(nominal)

    robot_pose = (0.5, 1.0, 0.0)
    obstacles = [(0.8, 1.1)]

    t0 = time.perf_counter()
    for _ in range(1000):
        path = avoid.select_path(obstacles, robot_pose)
    t_switch = (time.perf_counter() - t0) / 1000

    print(f"\nPath switch time (avg of 1000): {t_switch * 1e6:.1f} us")
    print(f"  Selected path: {avoid.active_path}")
    print(f"  Path length: {len(path)} pts")

    if t_switch < 0.001:
        print("PASS: Switch time under 1 ms")
    else:
        print(f"WARNING: Switch time is {t_switch*1000:.2f} ms")


def test_avoid():
    container = DestinationContainer()
    nominal_path_data = generate_nominal_path()
    path_for_test = pickle.dumps(nominal_path_data)

    container = DestinationContainer
    container.__reduce__
    with open("F:\\WRO\\World-Robot-Olympiad-2026\\history\\v6.9", "rb") as f:
        metadata = pickle.load(f)

    return metadata.get("name")
