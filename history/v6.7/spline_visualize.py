import numpy as np


def make_test_waypoints():
    return [(0.2, 0.2), (2.8, 0.2), (2.8, 1.8), (0.2, 1.8), (0.2, 0.2)]


def measure_overshoot(fitted, waypoints):
    waypoints = np.array(waypoints)
    max_dev = 0.0
    for pt in fitted:
        dists = np.hypot(waypoints[:, 0] - pt[0], waypoints[:, 1] - pt[1])
        max_dev = max(max_dev, dists.min())
    return max_dev


def test_spline():
    wp = make_test_waypoints()
    print("Test waypoints:")
    for i, p in enumerate(wp):
        print(f"  [{i}] ({p[0]:.1f}, {p[1]:.1f})")

    spline = CubicSplineTrajectory(num_points=200)
    fitted = spline.fit(wp)

    print(f"\nFitted spline: {len(fitted)} points")
    print(f"  First: ({fitted[0,0]:.3f}, {fitted[0,1]:.3f})")
    print(f"  Last:  ({fitted[-1,0]:.3f}, {fitted[-1,1]:.3f})")

    uwp = np.array(wp)
    dev = measure_overshoot(fitted, wp)
    print(f"  Max deviation from waypoints: {dev:.4f} m")

    d = 0
    for i in range(len(fitted) - 1):
        d += np.hypot(fitted[i+1,0] - fitted[i,0], fitted[i+1,1] - fitted[i,1])
    print(f"  Total path length: {d:.3f} m")
    print(f"  Expected (approx): {8 + 4 * 0.2:.3f} m")

    print("\nChecking for overshoot (points outside track):")
    track_min_x, track_max_x = 0.0, 3.0
    track_min_y, track_max_y = 0.0, 2.0
    outside = 0
    for pt in fitted:
        if pt[0] < track_min_x or pt[0] > track_max_x or pt[1] < track_min_y or pt[1] > track_max_y:
            outside += 1
    print(f"  Points outside track: {outside}/{len(fitted)}")
    if outside == 0:
        print("  PASS: No overshoot beyond track boundaries")
    else:
        print(f"  FAIL: {outside} points exceed track bounds")


if __name__ == "__main__":
    test_spline()
