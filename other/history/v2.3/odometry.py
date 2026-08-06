import json
import serial
import time

TICKS_PER_REV = 4095
WHEEL_RADIUS = 0.0325
WHEEL_CIRCUMFERENCE = 2 * 3.14159 * WHEEL_RADIUS
TRACK_WIDTH = 0.16

class Odometry:
    def __init__(self, uart):
        self.uart = uart
        self.left_ticks = 0
        self.right_ticks = 0
        self.distance = 0.0
        self.heading = 0.0
        self.last_time = time.time()

    def update(self):
        self.uart.write(json.dumps({"cmd": "poll_odometry"}).encode() + b'\n')
        line = self.uart.readline()
        if not line:
            return
        try:
            data = json.loads(line.decode().strip())
        except (json.JSONDecodeError, ValueError):
            return

        dl = data.get("left_delta", 0)
        dr = data.get("right_delta", 0)
        self.left_ticks += dl
        self.right_ticks += dr

        dist_delta = (dl + dr) / 2.0 / TICKS_PER_REV * WHEEL_CIRCUMFERENCE
        heading_delta = (dr - dl) / TICKS_PER_REV * WHEEL_CIRCUMFERENCE / TRACK_WIDTH

        self.distance += dist_delta
        self.heading += heading_delta

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

    def reset(self):
        self.left_ticks = 0
        self.right_ticks = 0
        self.distance = 0.0
        self.heading = 0.0

    def get_pose(self):
        return {
            "x": self.distance,
            "y": 0.0,
            "heading": self.heading,
            "left_ticks": self.left_ticks,
            "right_ticks": self.right_ticks,
        }
