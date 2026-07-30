import numpy as np
from .runner import SelfTestRunner, TestResult


def register_imu_tests(runner: SelfTestRunner, imu):
    def test_imu_init():
        if imu is None:
            return TestResult("imu_init").skipped("IMU disabled")
        imu.init()
        return TestResult("imu_init").passed()

    def test_imu_read():
        if imu is None:
            return TestResult("imu_read").skipped("IMU disabled")
        data = imu.read()
        if data is None:
            return TestResult("imu_read").failed("No data")
        accel, gyro = data["accel"], data["gyro"]
        if np.linalg.norm(accel) < 5 or np.linalg.norm(accel) > 15:
            return TestResult("imu_read").failed(f"Accel magnitude: {np.linalg.norm(accel):.1f}")
        return TestResult("imu_read").passed(
            f"accel=({accel[0]:.1f},{accel[1]:.1f},{accel[2]:.1f}) "
            f"gyro=({gyro[0]:.1f},{gyro[1]:.1f},{gyro[2]:.1f})",
            data=data
        )

    def test_imu_calibration():
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
