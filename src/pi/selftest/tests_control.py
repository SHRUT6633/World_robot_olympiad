import numpy as np
from .runner import SelfTestRunner, TestResult


def register_control_tests(runner: SelfTestRunner, stanley, servo_pid, motor_pid, kinematics):
    # Register four control-system self-tests:
    #   1. stanley_controller  -- compute steering for a straight-ahead scenario.
    #   2. servo_pid           -- compute servo angle from a small crosstrack error.
    #   3. motor_pid           -- compute motor speed from a velocity command.
    #   4. kinematics          -- dead-reckon one step with zero steering.
    #
    # stanley    -- instance of pi.control.stanley.StanleyController.
    # servo_pid  -- instance of pi.control.servo.ServoPID.
    # motor_pid  -- instance of pi.control.motor.MotorPID.
    # kinematics -- instance of pi.control.kinematics.Kinematics.

    def test_stanley():
        # Test the Stanley steering controller with zero initial error
        # (robot at (0,0) heading 0, target at (1,0) heading 0, speed 1 m/s).
        # The computed steering angle must not exceed the configured maximum.
        if stanley is None:
            return TestResult("stanley").skipped("Stanley disabled")
        steer = stanley.compute(0, 0, 0, 1, 0, 0, 1.0)
        if abs(steer) > stanley.max_steering:
            return TestResult("stanley").failed(
                f"Steering exceeds limit: {np.degrees(steer):.1f}deg"
            )
        return TestResult("stanley").passed(f"steer={np.degrees(steer):.2f}deg")

    def test_servo_pid():
        # Test the servo PID with a 0.5 rad cross-track error.
        # The resulting angle must be within the servo's physical range.
        if servo_pid is None:
            return TestResult("servo_pid").skipped("Servo PID disabled")
        angle = servo_pid.compute_angle(0.5, 0.0)
        if angle < servo_pid.min_angle or angle > servo_pid.max_angle:
            return TestResult("servo_pid").failed(f"Angle out of range: {angle:.1f}")
        return TestResult("servo_pid").passed(f"angle={angle:.2f}deg")

    def test_motor_pid():
        # Test the motor PID with a full-speed command (1.0).
        # The computed speed must be non-negative and below max_speed.
        if motor_pid is None:
            return TestResult("motor_pid").skipped("Motor PID disabled")
        speed = motor_pid.compute_speed(1.0, 0.0)
        if speed < 0 or speed > motor_pid.max_speed:
            return TestResult("motor_pid").failed(f"Speed out of range: {speed}")
        return TestResult("motor_pid").passed(f"speed={speed:.0f}")

    def test_kinematics():
        # Test the kinematic model with zero steering and forward motion.
        # Starting from (0, 0, 0), after 1.0 m/s for 0.1 s the displacement
        # should be very small (~0.1 m forward in X).
        if kinematics is None:
            return TestResult("kinematics").skipped("Kinematics disabled")
        x, y, h = kinematics.update(0, 0, 0, 1.0, 0.1, 0.01)
        if abs(x) > 0.1 or abs(y) > 0.1:
            return TestResult("kinematics").failed(
                f"Unexpected position: ({x:.4f},{y:.4f})"
            )
        return TestResult("kinematics").passed(f"({x:.4f},{y:.4f}) h={h:.4f}")

    runner.add("stanley_controller", test_stanley)
    runner.add("servo_pid", test_servo_pid)
    runner.add("motor_pid", test_motor_pid)
    runner.add("kinematics", test_kinematics)
