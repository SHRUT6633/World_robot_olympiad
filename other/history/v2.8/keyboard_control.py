import json
import serial
import time
import threading
from pynput import keyboard

PORT = '/dev/ttyAMA0'
BAUD = 115200

SPEED_NORMAL = 50
SPEED_SLOW = 20
SPEED_TURBO = 100
STEER_ANGLE = 30

class KeyboardControl:
    def __init__(self, port=PORT, baud=BAUD):
        self.uart = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2)
        self.current_keys = set()
        self.running = False
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )

    def on_press(self, key):
        try:
            self.current_keys.add(key.vk if hasattr(key, 'vk') else key.char)
        except AttributeError:
            self.current_keys.add(key)

    def on_release(self, key):
        try:
            self.current_keys.discard(key.vk if hasattr(key, 'vk') else key.char)
        except AttributeError:
            self.current_keys.discard(key)

    def send_command(self, speed, steer):
        msg = json.dumps({"cmd": "drive", "speed": speed, "steer": steer}) + '\n'
        self.uart.write(msg.encode())

    def get_speed(self):
        shift = any(k in self.current_keys for k in [
            keyboard.Key.shift, keyboard.Key.shift_r
        ])
        ctrl = any(k in self.current_keys for k in [
            keyboard.Key.ctrl, keyboard.Key.ctrl_r
        ])
        return SPEED_TURBO if shift else (SPEED_SLOW if ctrl else SPEED_NORMAL)

    def run(self):
        self.running = True
        self.listener.start()
        try:
            while self.running:
                speed = 0
                steer = 0
                if ord('w') in self.current_keys or 119 in self.current_keys:
                    speed = self.get_speed()
                elif ord('s') in self.current_keys or 115 in self.current_keys:
                    speed = -self.get_speed()
                if ord('a') in self.current_keys or 97 in self.current_keys:
                    steer = STEER_ANGLE
                elif ord('d') in self.current_keys or 100 in self.current_keys:
                    steer = -STEER_ANGLE
                if ord(' ') in self.current_keys or 32 in self.current_keys:
                    speed = 0
                    steer = 0
                    self.send_command(0, 0)
                else:
                    self.send_command(speed, steer)
                time.sleep(0.02)
        finally:
            self.send_command(0, 0)
            self.uart.close()

    def stop(self):
        self.running = False

if __name__ == '__main__':
    ctrl = KeyboardControl()
    ctrl.run()
