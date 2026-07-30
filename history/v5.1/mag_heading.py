import math
import time


class MagHeading:
    def __init__(self, alpha=0.98, mag_correction_gain=0.1):
        self.alpha = alpha
        self.mag_correction_gain = mag_correction_gain
        self.gyro_heading = 0.0
        self.filtered_heading = 0.0
        self.motor_running = False
        self._motor_stop_time = None
        self._last_update = None

    def update(self, gyro_yaw_rate: float, mag_heading: float,
               motor_running: bool, dt: float = None) -> float:
        now = time.monotonic()
        if self._last_update is None:
            self._last_update = now
            self.filtered_heading = mag_heading
            self.gyro_heading = mag_heading
            return self.filtered_heading

        if dt is None:
            dt = now - self._last_update
        self._last_update = now
        self.motor_running = motor_running
        self.gyro_heading = (self.gyro_heading + gyro_yaw_rate * dt) % (2 * math.pi)

        if motor_running:
            self._motor_stop_time = None
            self.filtered_heading = self.gyro_heading
        else:
            if self._motor_stop_time is None:
                self._motor_stop_time = now
            elapsed = now - self._motor_stop_time
            ramp = min(elapsed / 0.5, 1.0)
            gain = self.mag_correction_gain * ramp
            correction = (mag_heading - self.gyro_heading) * gain
            corrected = (self.gyro_heading + correction) % (2 * math.pi)
            self.filtered_heading = corrected

        return self.filtered_heading

    def heading_deg(self) -> float:
        return math.degrees(self.filtered_heading)
