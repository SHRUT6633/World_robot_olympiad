import time
import json
import serial

class TrajectoryOpen:
    def __init__(self, uart):
        self.uart = uart

    def send_command(self, speed, steer):
        msg = json.dumps({"cmd": "drive", "speed": speed, "steer": steer}) + '\n'
        self.uart.write(msg.encode())

    def execute(self, trajectory):
        cumulative = 0
        cum_times = []
        for pt in trajectory:
            cumulative += pt["duration_ms"]
            cum_times.append(cumulative)

        start = time.time()
        index = 0
        while index < len(trajectory):
            elapsed = (time.time() - start) * 1000.0
            if elapsed >= cum_times[index]:
                pt = trajectory[index]
                self.send_command(pt["speed"], pt["steer"])
                index += 1
            time.sleep(0.01)

    def build_turn_right(self):
        return [
            {"speed": 40, "steer": 0,  "duration_ms": 1000},
            {"speed": 30, "steer": -30, "duration_ms": 800},
            {"speed": 40, "steer": 0,  "duration_ms": 500},
            {"speed": 0,  "steer": 0,  "duration_ms": 0},
        ]

    def build_straight(self, speed=40, distance_m=2.0):
        approx_ms = int((distance_m / (speed * 0.025)) * 1000)
        return [
            {"speed": speed, "steer": 0, "duration_ms": approx_ms},
            {"speed": 0,     "steer": 0, "duration_ms": 0},
        ]
