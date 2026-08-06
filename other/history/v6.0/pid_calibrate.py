import time
import sys


def calibrate_speed_map(motor, encoder):
    measurements = []
    for pwm in range(30, 256, 10):
        motor.set_pwm(pwm)
        time.sleep(0.5)
        speeds = []
        for _ in range(20):
            speeds.append(encoder.read_speed())
            time.sleep(0.01)
        avg_speed = sum(speeds) / len(speeds)
        measurements.append((pwm, avg_speed))
        print(f"PWM={pwm:3d}  ->  speed={avg_speed:.3f} m/s")
    motor.set_pwm(0)
    return measurements


def plot_speed_map(measurements):
    try:
        import matplotlib.pyplot as plt
        pwms, speeds = zip(*measurements)
        plt.plot(pwms, speeds, "o-")
        plt.xlabel("PWM duty cycle")
        plt.ylabel("Speed (m/s)")
        plt.title("Open-loop PWM vs Speed")
        plt.grid(True)
        plt.savefig("speed_map.png")
        print("Saved speed_map.png")
    except ImportError:
        print("matplotlib not available, skipping plot")
