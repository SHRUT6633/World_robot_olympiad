import time
import math


def simulate_motor_interference(throttle: float) -> float:
    return 25.0 * throttle + 2.0 * (0.5 - hash(throttle) % 100 / 100.0)


def run_test():
    true_heading = 87.3
    for throttle in [0.0, 0.3, 0.5, 0.7, 1.0]:
        raw = true_heading + simulate_motor_interference(throttle)
        offset = raw - true_heading
        print(f"[MAG] Motors ON ({throttle*100:.0f}%): "
              f"heading {raw:.1f}\u00b0 (offset +{offset:.1f}\u00b0)"
              if throttle > 0 else
              f"[MAG] Static heading: {true_heading:.1f}\u00b0")
    print(f"[MAG] Motors OFF: heading {true_heading:.1f}\u00b0 (back to normal)")


if __name__ == "__main__":
    run_test()
