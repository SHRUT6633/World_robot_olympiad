import serial
import json
import time

PORT = '/dev/ttyAMA0'
BAUD = 115200

class DriveForward:
    def __init__(self, port=PORT, baud=BAUD):
        self.uart = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)

    def send_command(self, speed):
        msg = json.dumps({"cmd": "drive", "speed": speed}) + '\n'
        self.uart.write(msg.encode())

    def drive_forward(self, speed=50, duration=5):
        self.send_command(speed)
        time.sleep(duration)
        self.send_command(0)

    def close(self):
        self.uart.close()

if __name__ == '__main__':
    driver = DriveForward()
    try:
        driver.drive_forward(speed=50, duration=5)
    finally:
        driver.close()
