import time
import json
import serial

class PerformanceTest:
    def __init__(self, uart):
        self.uart = uart

    def send(self, cmd, **kwargs):
        msg = json.dumps({"cmd": cmd, **kwargs}) + '\n'
        self.uart.write(msg.encode())

    def read_odom(self):
        self.send("poll_odometry")
        line = self.uart.readline()
        if line:
            try:
                return json.loads(line.decode().strip())
            except (json.JSONDecodeError, ValueError):
                return None
        return None

    def speed_test(self, distance_m=10.0, speed=100):
        o = self.read_odom()
        start_dist = o["distance"] if o else 0.0
        start_time = time.time()

        self.send("drive", speed=speed, steer=0)
        while True:
            time.sleep(0.1)
            o = self.read_odom()
            if o is None:
                continue
            traveled = o["distance"] - start_dist
            if traveled >= distance_m:
                break
        elapsed = time.time() - start_time
        self.send("brake", duration_ms=50)

        avg_speed = distance_m / elapsed
        print(f"distance_m: {distance_m:.1f}")
        print(f"time_s: {elapsed:.2f}")
        print(f"avg_speed_ms: {avg_speed:.2f}")

    def turn_test(self, steer_angle=30, speed=30, duration=3):
        self.send("drive", speed=speed, steer=steer_angle)
        time.sleep(duration)
        o = self.read_odom()
        self.send("brake", duration_ms=50)
        return o

    def stop_test(self, speed=100):
        self.send("drive", speed=speed, steer=0)
        time.sleep(3)
        o_before = self.read_odom()
        self.send("brake", duration_ms=50)
        time.sleep(1)
        o_after = self.read_odom()
        if o_before and o_after:
            coast = o_after["distance"] - o_before["distance"]
            print(f"stop_distance_m: {coast:.3f}")
        return coast

    def full_test_suite(self):
        print("=== Performance Test Suite ===")
        self.speed_test(distance_m=10.0, speed=100)
        self.turn_test(steer_angle=30, speed=30, duration=3)
        self.stop_test(speed=100)
        print("=== All tests complete ===")
