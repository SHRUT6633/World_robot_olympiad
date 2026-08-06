import json
import serial
import time

from ackermann import AckermannSteering
from pid_straight import PIDStraight
from odometry import Odometry
from gyro_reader import GyroReader
from speed_ramp import SpeedRamp
from dynamic_brake import DynamicBrake

class FinalDrive:
    def __init__(self, port='/dev/ttyAMA0', baud=115200):
        self.uart = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2)
        self.steering = AckermannSteering(0.260, 0.160)
        self.pid = PIDStraight()
        self.odom = Odometry(self.uart)
        self.gyro = GyroReader(self.uart)
        self.ramp = SpeedRamp(self.uart)
        self.brake = DynamicBrake(self.uart)

    def drive(self, speed, steer_angle):
        pwm = self.steering.angle_to_pwm(steer_angle)
        msg = json.dumps({"cmd": "drive", "speed": speed, "steer": pwm}) + '\n'
        self.uart.write(msg.encode())

    def stop(self, brake=True):
        if brake:
            self.brake.brake(duration_ms=50)
        else:
            self.drive(0, 0)

    def get_odometry(self):
        self.odom.update()
        return self.odom.get_pose()

    def get_heading(self):
        self.gyro.poll()
        return self.gyro.get_heading()

    def close(self):
        self.drive(0, 0)
        self.uart.close()
