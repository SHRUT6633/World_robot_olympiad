import json
import serial
import time

class GyroReader:
    def __init__(self, uart):
        self.uart = uart
        self.heading = 0.0
        self.gyro_z = 0.0

    def poll(self):
        self.uart.write(json.dumps({"cmd": "poll_imu"}).encode() + b'\n')
        line = self.uart.readline()
        if not line:
            return
        try:
            data = json.loads(line.decode().strip())
        except (json.JSONDecodeError, ValueError):
            return
        self.heading = data.get("heading", self.heading)
        self.gyro_z = data.get("gyro_z", self.gyro_z)

    def get_heading(self):
        return self.heading

    def get_gyro_z(self):
        return self.gyro_z
