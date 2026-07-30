import numpy as np
from .runner import SelfTestRunner, TestResult


def register_mag_tests(runner: SelfTestRunner, mag):
    def test_mag_init():
        if mag is None:
            return TestResult("mag_init").skipped("Magnetometer disabled")
        mag.init()
        return TestResult("mag_init").passed()

    def test_mag_read():
        if mag is None:
            return TestResult("mag_read").skipped("Magnetometer disabled")
        data = mag.read_raw()
        if data is None:
            return TestResult("mag_read").failed("No data")
        magnitude = np.linalg.norm(data)
        if magnitude < 10 or magnitude > 1000:
            return TestResult("mag_read").failed(f"Magnitude: {magnitude:.0f}")
        return TestResult("mag_read").passed(
            f"mag=({data[0]:.0f},{data[1]:.0f},{data[2]:.0f})",
            data=data
        )

    def test_mag_calibration():
        if mag is None:
            return TestResult("mag_calib").skipped("Magnetometer disabled")
        mag.calibrate_hard_iron(samples=100)
        hard = mag.hard_iron
        return TestResult("mag_calib").passed(
            f"HardIron=({hard[0]:.0f},{hard[1]:.0f},{hard[2]:.0f})",
            data=hard
        )

    runner.add("mag_init", test_mag_init)
    runner.add("mag_read", test_mag_read)
    runner.add("mag_calib", test_mag_calibration)
