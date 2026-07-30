import numpy as np
from outlier_reject import OutlierRejectUKF


def run_test(inject_outlier: bool = True):
    ukf = OutlierRejectUKF()
    for i in range(200):
        dt = 0.02
        ukf.predict(dt)
        if i % 5 == 0:
            true = np.array([0.1 * i * dt, 0.0, 0.0])
            z = true + np.random.normal(0, 0.05, 3)
            if inject_outlier and i == 50:
                z[0] += 0.5
                print(f"[TEST] Injected outlier at step {i}: x={z[0]:.3f}")
            ukf.correct(z)
            err = np.hypot(ukf.ukf.x[0] - true[0], ukf.ukf.x[1] - true[1])
            if i == 52:
                print(f"[TEST] Post-outlier error: {err*100:.1f}cm "
                      f"{'<-- GOOD (rejected)' if err < 0.05 else '<-- BAD'}")


if __name__ == "__main__":
    run_test()
