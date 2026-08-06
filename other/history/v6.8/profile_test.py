import numpy as np


def make_oval_path():
    pts = []
    for t in np.linspace(0, 2 * np.pi, 200):
        x = 1.5 + 0.8 * np.cos(t)
        y = 1.0 + 0.5 * np.sin(t)
        pts.append((x, y))
    return np.array(pts)


def simulate_drive(path, v_profile, dt=0.01):
    x, y, heading = path[0, 0], path[0, 1], 0.0
    n = len(path)
    actual_speeds = []
    actual_accels = []

    for i in range(1, n):
        target_v = v_profile[i]

        dx = path[i, 0] - x
        dy = path[i, 1] - y
        target_heading = np.arctan2(dy, dx)
        heading_error = target_heading - heading
        heading += heading_error * 0.1

        x += target_v * np.cos(heading) * dt
        y += target_v * np.sin(heading) * dt

        actual_speeds.append(target_v)
        if len(actual_speeds) > 1:
            actual_accels.append((actual_speeds[-1] - actual_speeds[-2]) / dt)

    return actual_speeds, actual_accels


def test_profile():
    profiler = VelocityProfiler(max_v=2.0, max_a=0.5)

    path = make_oval_path()
    print(f"Path: {len(path)} points")

    v_profile = profiler.compute(path)
    print(f"Speed range: {v_profile.min():.3f} - {v_profile.max():.3f} m/s")
    print(f"Mean speed:  {v_profile.mean():.3f} m/s")

    speeds, accels = simulate_drive(path, v_profile)

    max_accel = max(accels) if accels else 0
    min_accel = min(accels) if accels else 0
    print(f"\nSimulated acceleration range: {min_accel:.2f} to {max_accel:.2f} m/s²")
    print(f"Max absolute acceleration: {max(abs(min_accel), abs(max_accel)):.2f} m/s²")

    max_abs = max(abs(min_accel), abs(max_accel))
    if max_abs <= 0.55:
        print(f"PASS: Max acceleration {max_abs:.2f} <= 0.5 m/s² (within tolerance)")
    else:
        print(f"WARNING: Max acceleration {max_abs:.2f} exceeds 0.5 m/s² limit")

    print("\nCurvature-limited speed check:")
    curvatures = profiler.curvature(path)
    max_curv = np.max(np.abs(curvatures))
    min_speed_idx = np.argmin(v_profile)
    print(f"  Max path curvature: {max_curv:.2f} 1/m")
    print(f"  Min speed at curvature peak: {v_profile[min_speed_idx]:.3f} m/s")
    expected_min = np.sqrt(2.0 / max_curv)
    print(f"  Expected min from lateral accel: {expected_min:.3f} m/s")


if __name__ == "__main__":
    test_profile()
