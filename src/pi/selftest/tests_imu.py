import numpy as np
from .runner import SelfTestRunner, TestResult


def register_imu_tests(runner: SelfTestRunner, imu):
    # Register three IMU self-tests:
    #   1. imu_init    -- initialise the IMU sensor.
    #   2. imu_read    -- read accelerometer and gyroscope data.
    #   3. imu_calib   -- calibrate gyroscope bias.
    #
    # imu -- an object implementing init(), read(), and calibrate_gyro_bias(),
    #        typically from pi.hardware.imu.

    def test_imu_init():
        # Test: initialise the IMU.
        # Returns SKIP if imu is None (feature disabled).
        if imu is None:
            return TestResult("imu_init").skipped("IMU disabled")
        imu.init()
        return TestResult("imu_init").passed()

    def test_imu_read():
        # Test: read a single IMU sample.
        # Accelerometer magnitude must be between 5 and 15 m/s^2
        # (Earth's gravity is ~9.81 m/s^2; this range allows for
        # moderate motion).
        if imu is None:
            return TestResult("imu_read").skipped("IMU disabled")
        data = imu.read()
        if data is None:
            return TestResult("imu_read").failed("No data")
        accel, gyro = data["accel"], data["gyro"]
        if np.linalg.norm(accel) < 5 or np.linalg.norm(accel) > 15:
            return TestResult("imu_read").failed(
                f"Accel magnitude: {np.linalg.norm(accel):.1f}"
            )
        return TestResult("imu_read").passed(
            f"accel=({accel[0]:.1f},{accel[1]:.1f},{accel[2]:.1f}) "
            f"gyro=({gyro[0]:.1f},{gyro[1]:.1f},{gyro[2]:.1f})",
            data=data,
        )

    def test_imu_calibration():
        # Test: calibrate the gyroscope bias using 50 samples.
        # The bias on each axis must be <= 10 degrees/s (0.1745 rad/s).
        if imu is None:
            return TestResult("imu_calib").skipped("IMU disabled")
        imu.calibrate_gyro_bias(samples=50)
        bias = imu.gyro_bias
        if np.any(np.abs(bias) > 10):
            return TestResult("imu_calib").failed(f"Gyro bias too high: {bias}")
        return TestResult("imu_calib").passed(f"Bias={bias}", data=bias)

    runner.add("imu_init", test_imu_init)
    runner.add("imu_read", test_imu_read)
    runner.add("imu_calib", test_imu_calibration)
