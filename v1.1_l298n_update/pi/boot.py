"""
WRO 4WS Boot Sequence
======================
Flow:
  1. Power ON → All LEDs flash once
  2. Run comprehensive self-test on ALL modules
  3. If ALL pass → Green LED ON steady → wait for switch
  4. If ANY fail → Red LED blinks → log errors → halt
  5. Switch pressed → Full race logic begins
"""

import time
import sys
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from pi.system.logger import Logger, log as logger
from pi.system.config_manager import ConfigManager
from pi.hardware.led import StatusLED
from pi.hardware.switch import StartSwitch
from pi.selftest import (
    SelfTestRunner, TestResult,
    register_camera_tests, register_tof_tests,
    register_imu_tests, register_mag_tests,
    register_fusion_tests, register_control_tests,
    register_comm_tests,
)


BOOT_VERSION = "WRO_4WS_BOOT_v1.0"


def power_on_self_test():
    logger.info("=" * 50)
    logger.info(f"  {BOOT_VERSION}")
    logger.info("  Power-On Self-Test: ALL MODULES")
    logger.info("=" * 50)

    config = ConfigManager()
    config.load()

    runner = SelfTestRunner()

    from pi.sensors.camera.camera_driver import PiCamera
    from pi.sensors.tof.vl53l0x import VL53L0X
    from pi.sensors.tof.vl53l1x import VL53L1X
    from pi.sensors.imu.mpu6050 import MPU6050
    from pi.sensors.magnetometer.qmc5883l import QMC5883L
    from pi.fusion.ukf import RobotUKF
    from pi.fusion.complementary import ComplementaryFilter
    from pi.control.stanley import StanleyController
    from pi.control.servo_pid import ServoPID
    from pi.control.motor_pid import MotorPID
    from pi.dynamics.kinematic_model import KinematicModel
    from pi.comm.uart import UARTCommunicator

    camera = PiCamera()
    tof_left = VL53L0X(
        "left",
        xshut_pin=config.get("sensors", "vl53l0x_left", "xshut_pin", default=None),
    )
    tof_right = VL53L0X(
        "right",
        xshut_pin=config.get("sensors", "vl53l0x_right", "xshut_pin", default=None),
    )
    tof_front = VL53L1X(
        "front",
        xshut_pin=config.get("sensors", "vl53l1x_front", "xshut_pin", default=None),
    )
    imu = MPU6050()
    mag = QMC5883L()
    ukf = RobotUKF(dt=0.01)
    comp_filter = ComplementaryFilter()
    stanley = StanleyController()
    servo_pid = ServoPID()
    motor_pid = MotorPID()
    kinematics = KinematicModel()
    uart = UARTCommunicator()

    register_camera_tests(runner, camera)
    register_tof_tests(runner, {
        "left": tof_left, "right": tof_right, "front": tof_front
    })
    register_imu_tests(runner, imu)
    register_mag_tests(runner, mag)
    register_fusion_tests(runner, ukf, comp_filter)
    register_control_tests(runner, stanley, servo_pid, motor_pid, kinematics)
    register_comm_tests(runner, uart)

    results = runner.run_all()

    return results, runner


def boot_sequence():
    config = ConfigManager()
    config.load()

    led = StatusLED(
        green_pin=config.get("hardware", "leds", "green_pin", default=23),
        red_pin=config.get("hardware", "leds", "red_pin", default=24),
    )
    switch = StartSwitch(
        pin=config.get("hardware", "switch", "pin", default=25),
    )

    led.blink_pattern(["green", "red", "off"], interval=0.15)

    results, runner = power_on_self_test()

    if results["failed"] == 0:
        led.success_sequence()
        logger.info("")
        logger.info("=" * 50)
        logger.info("  ALL TESTS PASSED  ")
        logger.info("  Green LED ON - Press switch to start race")
        logger.info("=" * 50)

        pressed = switch.wait_for_press(timeout=None)
        if pressed:
            logger.info("Starting race logic...")
            led.stop_blink()
            led.set("blue")
            return True
    else:
        led.error_sequence()
        logger.info("")
        logger.info("=" * 50)
        logger.info("  SELF-TEST FAILED  ")
        logger.info(f"  {results['failed']} test(s) failed")
        logger.info("  Check logs for details")
        logger.info("=" * 50)
        return False


def main():
    logger.init(name="WRO_BOOT", level="INFO", log_dir="logs")
    logger.info("WRO 4WS Booting...")

    try:
        success = boot_sequence()
        if success:
            logger.info("Launching main race program...")
            from pi.main import main as race_main
            import asyncio
            asyncio.run(race_main())
        else:
            logger.critical("Boot aborted due to test failures")
            sys.exit(1)
    except KeyboardInterrupt:
        logger.info("Boot interrupted")
    except Exception as e:
        logger.critical(f"Boot fatal: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
