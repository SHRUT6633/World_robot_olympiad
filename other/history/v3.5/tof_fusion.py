import time
import numpy as np
from collections import namedtuple
from read_tof import read_left, read_right, read_front, left, right, front, GPIO

WallState = namedtuple("WallState", ["left_mm", "right_mm", "front_mm",
                                      "wall_dist_mm", "wall_angle_rad"])

SENSOR_SPACING_MM = 160.0
FRONT_WINDOW_SIZE = 5

front_buffer = None

def init_front_buffer(initial_mm):
    global front_buffer
    front_buffer = [initial_mm] * FRONT_WINDOW_SIZE

def read_all_staggered():
    left_d = read_left()
    time.sleep(0.020)
    right_d = read_right()
    time.sleep(0.020)
    front_d = read_front()
    return left_d, right_d, front_d

def fuse_tof():
    global front_buffer
    left_d, right_d, front_d = read_all_staggered()

    if front_buffer is None:
        init_front_buffer(front_d)

    front_buffer.pop(0)
    front_buffer.append(front_d)
    front_smooth = int(np.mean(front_buffer))

    wall_dist = (left_d + right_d) / 2.0
    wall_angle = np.arctan2(right_d - left_d, SENSOR_SPACING_MM)

    return WallState(left_mm=left_d, right_mm=right_d,
                     front_mm=front_smooth,
                     wall_dist_mm=wall_dist,
                     wall_angle_rad=wall_angle)

if __name__ == "__main__":
    init_front_buffer(500)
    for i in range(50):
        s = fuse_tof()
        print(f"wall_dist={s.wall_dist_mm:.0f}mm  "
              f"angle={np.rad2deg(s.wall_angle_rad):.1f}deg  "
              f"front={s.front_mm}mm")
        time.sleep(0.060)

    left.stop_ranging()
    right.stop_ranging()
    front.stop_ranging()
    GPIO.cleanup()
