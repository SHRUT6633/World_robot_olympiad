import time

class PIDStraight:
    def __init__(self, kp=7.2, ki=28.8, kd=0.0, integral_limit=50.0, output_limit=255.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, heading, target_heading, speed):
        error = target_heading - heading
        dt = time.time() - self.last_time
        if dt <= 0.0:
            dt = 0.02

        if abs(error) < 10.0:
            self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        self.last_time = time.time()

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))

        left = int(speed - output)
        right = int(speed + output)
        left = max(-100, min(100, left))
        right = max(-100, min(100, right))
        return left, right, error
