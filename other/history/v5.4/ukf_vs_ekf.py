import math
import numpy as np
from ukf_localization import UKF


def run_comparison():
    ukf = UKF()
    dt = 0.02
    ukf_errors = []
    for t in np.arange(0, 5.0, dt):
        true_x = 0.5 * math.sin(t * 0.5)
        true_y = 0.5 * (1 - math.cos(t * 0.5))
        ukf.predict(dt)
        if int(t / 0.2) % 5 == 0:
            z = np.array([true_x, true_y, t * 0.5])
            ukf.correct(z)
            error = math.hypot(ukf.ukf.x[0] - true_x,
                               ukf.ukf.x[1] - true_y)
            ukf_errors.append(error)
    print(f"[COMPARISON] UKF mean error: "
          f"{np.mean(ukf_errors)*100:.1f}cm")


if __name__ == "__main__":
    run_comparison()
