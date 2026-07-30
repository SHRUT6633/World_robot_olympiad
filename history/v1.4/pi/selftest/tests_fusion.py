import numpy as np
from .runner import SelfTestRunner, TestResult


def register_fusion_tests(runner: SelfTestRunner, ukf, comp_filter):
    # Register three sensor-fusion self-tests:
    #   1. ukf_predict        -- run one UKF prediction step, check state dim.
    #   2. ukf_update         -- run one UKF update with a synthetic measurement.
    #   3. comp_filter        -- run one complementary filter update with
    #                            synthetic accelerometer/gyro data.
    #
    # ukf          -- instance of pi.fusion.ukf.UnscentedKalmanFilter.
    # comp_filter  -- instance of pi.fusion.complementary.ComplementaryFilter.

    def test_ukf_predict():
        # Test: execute a single UKF prediction step.
        # The state vector should have exactly 6 elements:
        #   [x, y, heading, vx, vy, omega].
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
        # Test: execute a UKF update with a synthetic measurement.
        # z = [x, y, heading, vx, vy, omega] all near zero.
        if ukf is None:
            return TestResult("ukf_update").skipped("UKF disabled")
        z = np.array([0.1, 0.2, 0.05, 0.5, 0.0, 0.0])
        ukf.update(z)
        state = ukf.state
        return TestResult("ukf_update").passed(
            f"x={state[0]:.3f} y={state[1]:.3f}"
        )

    def test_complementary():
        # Test the complementary filter with a stationary-like input:
        #   accel = (0, 0, 9.81) -- gravity only, no linear acceleration.
        #   gyro  = (0.01, 0.02, 0.005) -- very slow rotation.
        # The estimated pitch/roll should be near zero (within 0.5 rad).
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
