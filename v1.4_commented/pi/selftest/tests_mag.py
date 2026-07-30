import numpy as np
from .runner import SelfTestRunner, TestResult


def register_mag_tests(runner: SelfTestRunner, mag):
    # Register three magnetometer self-tests:
    #   1. mag_init   -- initialise the magnetometer.
    #   2. mag_read   -- read raw magnetic field vector.
    #   3. mag_calib  -- calibrate hard-iron offsets.
    #
    # mag -- an object implementing init(), read_raw(), and
    #        calibrate_hard_iron(), typically from pi.hardware.magnetometer.

    def test_mag_init():
        # Test: initialise the magnetometer.
        if mag is None:
            return TestResult("mag_init").skipped("Magnetometer disabled")
        mag.init()
        return TestResult("mag_init").passed()

    def test_mag_read():
        # Test: read a raw magnetic field sample.
        # The magnitude must be between 10 and 1000 uT (typical Earth field
        # is ~25-65 uT indoors, but can vary wildly near motors/ferrous
        # materials; wide bounds avoid false negatives).
        if mag is None:
            return TestResult("mag_read").skipped("Magnetometer disabled")
        data = mag.read_raw()
        if data is None:
            return TestResult("mag_read").failed("No data")
        magnitude = np.linalg.norm(data)
        if magnitude < 10 or magnitude > 1000:
            return TestResult("mag_read").failed(f"Magnitude: {magnitude:.0f}")
        return TestResult("mag_read").passed(
            f"mag=({data[0]:.0f},{data[1]:.0f},{data[2]:.0f})", data=data
        )

    def test_mag_calibration():
        # Test: calibrate hard-iron distortion by sampling 100 points.
        # The resulting offset vector is stored in mag.hard_iron.
        if mag is None:
            return TestResult("mag_calib").skipped("Magnetometer disabled")
        mag.calibrate_hard_iron(samples=100)
        hard = mag.hard_iron
        return TestResult("mag_calib").passed(
            f"HardIron=({hard[0]:.0f},{hard[1]:.0f},{hard[2]:.0f})", data=hard
        )

    runner.add("mag_init", test_mag_init)
    runner.add("mag_read", test_mag_read)
    runner.add("mag_calib", test_mag_calibration)
