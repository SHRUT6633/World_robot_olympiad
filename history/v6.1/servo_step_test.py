import time
import sys


class ServoStepTester:
    def __init__(self, servo_pid, adc_reader, pwm_writer):
        self.pid = servo_pid
        self.adc = adc_reader
        self.pwm = pwm_writer

    def step_test(self, angle, duration=1.0):
        print(f"Step test: {angle} degrees")
        t0 = time.time()
        readings = []
        while time.time() - t0 < duration:
            current = self.adc.read_angle()
            pulse = self.pid.compute(angle, current)
            self.pwm.set_pulse(pulse)
            readings.append((time.time() - t0, current))
            time.sleep(self.pid.dt)
        return readings

    def analyze(self, readings, target):
        if not readings:
            return
        current_vals = [r[1] for r in readings]
        peak = max(current_vals)
        overshoot = peak - target
        print(f"  Overshoot: {overshoot:.2f} deg")
        settled = None
        for r in readings:
            if abs(r[1] - target) < 1.0:
                settled = r[0]
                break
        if settled:
            print(f"  Settle time: {settled:.3f} s")
        else:
            print(f"  Did not settle within tolerance")


def run_tests():
    import sys
    print("Servo PID Step Response Test")
    print("Run this on the Pi with ADC connected to servo potentiometer")
    print("and servo signal wire connected to PWM output.")
    print("Usage: python servo_step_test.py")
    print("Make sure servo is powered and potentiometer is wired to ADC channel 0")
    print()

    pid = ServoPID(dt=0.01)

    from servo_hardware import ADCReader, PWMWriter
    adc = ADCReader(channel=0)
    pwm = PWMWriter(channel=0, freq=50)
    tester = ServoStepTester(pid, adc, pwm)

    for angle in [10, 20, -10, -20]:
        readings = tester.step_test(angle, duration=1.5)
        tester.analyze(readings, angle)


if __name__ == "__main__":
    run_tests()
