import math


def plot_waypoints(waypoints, label):
    print(f"\n=== {label} ({len(waypoints)} points) ===")
    if len(waypoints) <= 10:
        for i, wp in enumerate(waypoints):
            print(f"  [{i}] ({wp[0]:.3f}, {wp[1]:.3f})")
    else:
        print(f"  First 5 and last 5:")
        for i, wp in enumerate(waypoints[:5]):
            print(f"  [{i}] ({wp[0]:.3f}, {wp[1]:.3f})")
        print(f"  ...")
        for i, wp in enumerate(waypoints[-5:]):
            idx = len(waypoints) - 5 + i
            print(f"  [{idx}] ({wp[0]:.3f}, {wp[1]:.3f})")


def check_spacing(waypoints):
    if len(waypoints) < 2:
        return 0, 0, 0
    dists = []
    for i in range(len(waypoints) - 1):
        d = math.hypot(waypoints[i + 1][0] - waypoints[i][0], waypoints[i + 1][1] - waypoints[i][1])
        dists.append(d)
    return min(dists), max(dists), sum(dists) / len(dists)


def test_global_planner():
    planner = GlobalPlanner(center_offset=0.2)

    print("Global Planner Test")
    print("Track: 3.0m x 2.0m")

    planner.plan_rectangle(3.0, 2.0)
    plot_waypoints(planner.waypoints, "Base waypoints (no interpolation)")

    print("\nCorner-cutting analysis:")
    for i in range(len(planner.waypoints) - 1):
        p0 = planner.waypoints[i]
        p1 = planner.waypoints[i + 1]
        d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        print(f"  Segment {i}: distance = {d:.3f} m")

    planner.interpolate(spacing=0.1)
    plot_waypoints(planner.waypoints, "Interpolated (100mm spacing)")

    min_d, max_d, avg_d = check_spacing(planner.waypoints)
    print(f"\nSpacing: min={min_d:.3f} max={max_d:.3f} avg={avg_d:.3f} m")
    assert max_d < 0.15, f"Max spacing too large: {max_d}"
    print("OK: max spacing within 150mm")

    planner.reverse_direction()
    print(f"\nReversed direction: {len(planner.waypoints)} points")
    print(f"  First: ({planner.waypoints[0][0]:.1f}, {planner.waypoints[0][1]:.1f})")
    print(f"  Last:  ({planner.waypoints[-1][0]:.1f}, {planner.waypoints[-1][1]:.1f})")
