import math
import numpy as np
from ekf_localization import EKF


def run_test():
    ekf = EKF()
    dt = 0.02
    total_error = 0.0
    count = 0
    for t in np.arange(0, 5.0, dt):
        true_x = 0.5 * math.sin(t * 0.5)
        true_y = 0.5 * (1 - math.cos(t * 0.5))
        true_theta = t * 0.5

        if t < 2.0:
            v, omega = 0.3, 0.5
        else:
            v, omega = 0.0, math.radians(112)

        ekf.predict(v, omega, dt)
        if count % 10 == 0:
            z = np.array([true_x, true_y]) + np.random.normal(0, 0.05, 2)
            ekf.correct(z)
            error = math.hypot(ekf.x[0] - true_x, ekf.x[1] - true_y)
            total_error += error
            if error > 0.1:
                print(
                    f"[EKF] Sharp turn: innov=({z[0]-ekf.x[0]:.3f}, "
                    f"{z[1]-ekf.x[1]:.3f}) post-error={error*100:.1f}cm"
                )
        count += 1
    print(f"[EKF] Mean error: {total_error/(count//10)*100:.1f}cm")


if __name__ == "__main__":
    run_test()
