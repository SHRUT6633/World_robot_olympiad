import math
import numpy as np
from complementary_full import ComplementaryFull


def simulate_rotation(rate_dps: float, duration: float, dt: float = 0.01):
    cf = ComplementaryFull()
    cf.gyro_bias = np.zeros(3)
    rate_rad = math.radians(rate_dps)
    steps = int(duration / dt)
    max_error = 0.0
    for i in range(steps):
        true_yaw = (rate_rad * i * dt) % (2 * math.pi)
        accel = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, rate_rad])
        mag = np.array([1.0, 0.0, 0.0])
        r, p, y = cf.update(accel, gyro, mag, dt)
        error = abs(y - true_yaw)
        if error > max_error:
            max_error = error
    status = "OK" if max_error < math.radians(5) else "DIVERGED"
    print(f"[FILTER] Rotation rate: {rate_dps}\u00b0/s "
          f"\u2014 {status}, {math.degrees(max_error):.0f}\u00b0 error")
    return max_error


if __name__ == "__main__":
    for rate in [30, 60, 85, 95, 120]:
        simulate_rotation(rate, 5.0)
