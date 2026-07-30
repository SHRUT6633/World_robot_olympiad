import json
import serial
import time
import math

class StopReverse:
    def __init__(self, uart):
        self.uart = uart

    def send(self, cmd, **kwargs):
        msg = json.dumps({"cmd": cmd, **kwargs}) + '\n'
        self.uart.write(msg.encode())

    def drive_at_speed(self, speed, duration=2):
        self.send("drive", speed=speed, steer=0)
        time.sleep(duration)

    def stop_normal(self):
        self.send("drive", speed=0, steer=0)

    def stop_brake(self, brake_ms=50):
        self.send("brake", brake_ms=brake_ms)

    def stop_emergency(self):
        self.stop_brake(brake_ms=200)

    def drive_reverse(self, speed=30, duration=2):
        self.send("drive", speed=-speed, steer=0)
        time.sleep(duration)
        self.send("drive", speed=0, steer=0)

    def measure_coast(self, speed, brake=False):
        self.uart.flush()
        self.send("drive", speed=speed, steer=0)
        time.sleep(3)

        self.send("poll_odometry")
        line = self.uart.readline()
        before = self._parse_odom(line)

        if brake:
            self.stop_brake()
        else:
            self.stop_normal()

        time.sleep(1)
        self.send("poll_odometry")
        line = self.uart.readline()
        after = self._parse_odom(line)

        if before is not None and after is not None:
            coast = after["distance"] - before["distance"]
        else:
            coast = -1
        return coast

    def _parse_odom(self, line):
        if not line:
            return None
        try:
            return json.loads(line.decode().strip())
        except (json.JSONDecodeError, ValueError):
            return None

    def coast_test(self):
        print("speed_pct, coast_normal_m, coast_brake_m")
        for speed in [10, 25, 50, 75, 100]:
            n = self.measure_coast(speed, brake=False)
            b = self.measure_coast(speed, brake=True)
            print(f"{speed}, {n:.3f}, {b:.3f}")
