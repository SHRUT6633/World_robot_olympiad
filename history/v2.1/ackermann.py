import math

SERVO_CENTER = 1500
SERVO_RANGE = 500

class AckermannSteering:
    def __init__(self, wheelbase, track_width, servo_centre=SERVO_CENTER, servo_range=SERVO_RANGE):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.servo_centre = servo_centre
        self.servo_range = servo_range

    def ackermann_angles(self, radius):
        if radius == 0 or abs(radius) < self.track_width / 2:
            raise ValueError("Radius too small")
        inside = math.atan(self.wheelbase / (abs(radius) - self.track_width / 2))
        outside = math.atan(self.wheelbase / (abs(radius) + self.track_width / 2))
        if radius < 0:
            inside, outside = -inside, -outside
        return math.degrees(inside), math.degrees(outside)

    def effective_angle(self, radius):
        inside, outside = self.ackermann_angles(radius)
        return (inside + outside) / 2

    def theoretical_radius(self, steering_angle_deg):
        rad = math.radians(abs(steering_angle_deg))
        if math.tan(rad) < 1e-6:
            return float('inf')
        r = self.wheelbase / math.tan(rad)
        return r if steering_angle_deg >= 0 else -r

    def angle_to_pwm(self, angle_deg):
        fraction = angle_deg / 45.0
        fraction = max(-1.0, min(1.0, fraction))
        pwm = self.servo_centre + fraction * self.servo_range
        return int(pwm)

    def pwm_to_angle(self, pwm):
        fraction = (pwm - self.servo_centre) / self.servo_range
        return fraction * 45.0
