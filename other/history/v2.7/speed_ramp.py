from scurve_profile import SCurveProfile
import time

class SpeedRamp:
    def __init__(self, uart, ramp_time=0.5, loop_hz=100):
        self.uart = uart
        self.profile = SCurveProfile(ramp_time)
        self.dt = 1.0 / loop_hz

    def ramp_to(self, from_speed, to_speed, ramp_time=None):
        if ramp_time is not None:
            self.profile = SCurveProfile(ramp_time)
        start = time.time()
        while True:
            t = time.time() - start
            if t >= self.profile.ramp_time:
                break
            speed = self.profile.velocity(t, from_speed, to_speed)
            self._set_speed(int(speed))
            time.sleep(self.dt)
        self._set_speed(to_speed)

    def accelerate(self, to_speed, ramp_time=0.5):
        self.ramp_to(0, to_speed, ramp_time)

    def decelerate(self, from_speed, ramp_time=0.5):
        self.ramp_to(from_speed, 0, ramp_time)

    def _set_speed(self, speed):
        import json
        msg = json.dumps({"cmd": "drive", "speed": speed}) + '\n'
        self.uart.write(msg.encode())
