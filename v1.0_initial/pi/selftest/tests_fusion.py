import numpy as np
from .runner import SelfTestRunner, TestResult


def register_fusion_tests(runner: SelfTestRunner, ukf, comp_filter):
    def test_ukf_predict():
        if ukf is None:
            return TestResult("ukf_predict").skipped("UKF disabled")
        ukf.predict()
        state = ukf.state
        if len(state) != 6:
            return TestResult("ukf_predict").failed(f"Bad state dim: {len(state)}")
        return TestResult("ukf_predict").passed(
            f"x={state[0]:.3f} y={state[1]:.3f} h={state[2]:.3f}"
        )

    def test_ukf_update():
        if ukf is None:
            return TestResult("ukf_update").skipped("UKF disabled")
        z = np.array([0.1, 0.2, 0.05, 0.5, 0.0, 0.0])
        ukf.update(z)
        state = ukf.state
        return TestResult("ukf_update").passed(
            f"x={state[0]:.3f} y={state[1]:.3f}"
        )

    def test_complementary():
        if comp_filter is None:
            return TestResult("comp_filter").skipped("Complementary filter disabled")
        accel = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.01, 0.02, 0.005])
        pitch, roll, yaw = comp_filter.update(accel, gyro)
        if abs(pitch) > 0.5 and abs(roll) > 0.5:
            return TestResult("comp_filter").failed(
                f"Unexpected angles: pitch={pitch:.3f} roll={roll:.3f}"
            )
        return TestResult("comp_filter").passed(
            f"pitch={pitch:.3f} roll={roll:.3f} yaw={yaw:.3f}"
        )

    runner.add("ukf_predict", test_ukf_predict)
    runner.add("ukf_update", test_ukf_update)
    runner.add("comp_filter", test_complementary)
