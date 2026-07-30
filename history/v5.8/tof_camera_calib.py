import numpy as np


def calibrate_sensor_offset():
    measurements = [
        (0.50, 0.47),
        (1.00, 0.94),
        (1.50, 1.43),
        (2.00, 1.92),
    ]
    offsets = []
    for tof, cam in measurements:
        offsets.append(cam - tof)
    mean_offset = np.mean(offsets)
    print(f"[CALIB] ToF-to-camera offset: {mean_offset*100:.1f}cm "
          f"(camera reads {abs(mean_offset)*100:.1f}cm "
          f"{'closer' if mean_offset < 0 else 'farther'})")
    print(f"[CALIB] Recommend x_offset = {abs(mean_offset):.3f}m")
    return abs(mean_offset)


if __name__ == "__main__":
    calibrate_sensor_offset()
