import math
import time
import serial
import json

from ackermann import AckermannSteering

PORT = '/dev/ttyAMA0'
BAUD = 115200
WHEELBASE = 0.260
TRACK_WIDTH = 0.160

class TurnTest:
    def __init__(self):
        self.uart = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)
        self.steering = AckermannSteering(WHEELBASE, TRACK_WIDTH)

    def send_drive(self, speed, steering_angle):
        msg = json.dumps({"cmd": "drive", "speed": speed}) + '\n'
        self.uart.write(msg.encode())
        pwm = self.steering.angle_to_pwm(steering_angle)
        msg = json.dumps({"cmd": "steer", "pwm": pwm}) + '\n'
        self.uart.write(msg.encode())

    def measure_radius(self, angle_deg, speed=20, duration=3):
        self.send_drive(speed, angle_deg)
        time.sleep(duration)
        radius = self.steering.theoretical_radius(angle_deg)
        return radius

    def run_test(self):
        angles = [-30, -15, 0, 15, 30]
        print("angle_deg, theoretical_radius_m, note")
        for angle in angles:
            radius = self.measure_radius(angle)
            direction = "left" if angle > 0 else ("right" if angle < 0 else "straight")
            print(f"{angle}, {radius:.3f}, {direction}")
        self.send_drive(0, 0)

    def close(self):
        self.uart.close()

if __name__ == '__main__':
    test = TurnTest()
    try:
        test.run_test()
    finally:
        test.close()
