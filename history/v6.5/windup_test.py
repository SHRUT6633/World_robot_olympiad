import time


class SimplePID:
    def __init__(self, kp=0.5, ki=0.1, kd=0.01, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, limit=100, anti_windup=None):
        if anti_windup is not None:
            self.integral = anti_windup.conditional(self.integral, error, error, limit)
        else:
            self.integral += error * self.dt

        derivative = (error - self.last_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return max(-limit, min(limit, output))


def simulate_restrained_start(use_anti_windup):
    pid = SimplePID(kp=0.5, ki=0.1, dt=0.01)
    aw = AntiWindup(dt=0.01) if use_anti_windup else None

    speed = 0.0
    target = 1.0
    held = True
    log = []

    for t_ms in range(5000):
        t = t_ms * 0.01

        if t >= 3.0:
            held = False

        error = target - speed
        output = pid.compute(error, limit=255, anti_windup=aw)

        if held:
            speed = 0.0
        else:
            speed += output * 0.005
            speed = max(0.0, speed)

        if t_ms % 10 == 0:
            log.append((t, speed, pid.integral, error))

    return log


def print_results(log, label):
    print(f"\n=== {label} ===")
    print(f"{'Time':>6} {'Speed':>8} {'Integral':>10} {'Error':>8}")
    print("-" * 36)
    for t, speed, integral, error in log:
        if abs(t - round(t)) < 0.01 or abs(t - round(t * 10) / 10) < 0.01:
            print(f"{t:6.2f} {speed:8.3f} {integral:10.3f} {error:8.3f}")

    peak = max(l[1] for l in log)
    print(f"Peak speed: {peak:.3f} m/s")


log_no_windup = simulate_restrained_start(use_anti_windup=False)
print_results(log_no_windup, "NO ANTI-WINDUP")

log_with_windup = simulate_restrained_start(use_anti_windup=True)
print_results(log_with_windup, "WITH ANTI-WINDUP (conditional)")
