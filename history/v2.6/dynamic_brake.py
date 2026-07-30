import json
import serial
import time

class DynamicBrake:
    def __init__(self, uart):
        self.uart = uart
        self.brake_cooldown = 1.0
        self.last_brake_time = 0

    def brake(self, duration_ms=50):
        now = time.time()
        elapsed = now - self.last_brake_time
        if elapsed < self.brake_cooldown:
            print(f"brake cooldown: wait {self.brake_cooldown - elapsed:.1f}s")
            return

        msg = json.dumps({
            "cmd": "brake",
            "duration_ms": duration_ms,
            "polarity": "reverse"
        }) + '\n'
        self.uart.write(msg.encode())
        self.last_brake_time = time.time()

    def emergency_stop(self):
        self.brake(duration_ms=200)

    def release(self):
        msg = json.dumps({"cmd": "brake", "duration_ms": 0}) + '\n'
        self.uart.write(msg.encode())

class BrakeController:
    def __init__(self, gpio_enable, gpio_dir1, gpio_dir2, pwm_channel):
        self.enable = gpio_enable
        self.dir1 = gpio_dir1
        self.dir2 = gpio_dir2
        self.pwm = pwm_channel
        self.thermal_cooldown = 0

    def get_stop_distance(self, speed_pct):
        if speed_pct <= 10:
            return 0.02
        elif speed_pct <= 25:
            return 0.04
        elif speed_pct <= 50:
            return 0.07
        elif speed_pct <= 75:
            return 0.10
        else:
            return 0.12
