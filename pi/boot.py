# =============================================================================
# boot.py — WRO 4WS Robot Boot Sequence
# =============================================================================
# Flow (documented in the module docstring reproduced below):
#   1. Power ON → All LEDs flash once
#   2. Run comprehensive self-test on ALL modules
#   3. If ALL pass → Green LED ON steady → wait for switch
#   4. If ANY fail → Red LED blinks → log errors → halt
#   5. Switch pressed → Full race logic begins
#
# This module is intended to run ONCE at power-up (e.g., from rc.local or
# as a systemd service). It is the safety gate: if hardware self-tests fail,
# the robot never enters race mode, preventing runaway or damage.
#
# Key design decisions:
#   - Self-test instantiates FRESH copies of every driver (not reusing the
#     main.py instances) so that we can verify hardware independently before
#     committing to the full software stack.
#   - The start switch (physical button) is the final go/no-go. The robot
#     sits idle (green LED) until a human presses it.
#   - After the switch press, boot.py calls into pi.main.main() which builds
#     everything again for race mode. This double-instantiation is acceptable
#     because self-test objects go out of scope after POST.
# =============================================================================

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

# Add project root (one level above pi/) to sys.path so pi.xxx imports work.
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


# Version string logged at the start of POST for traceability.
BOOT_VERSION = "WRO_4WS_BOOT_v1.0"


# =============================================================================
# power_on_self_test()
# =============================================================================
# This function instantiates every hardware driver and every critical
# software module, registers them with a SelfTestRunner, and runs all tests.
#
# Called by: boot_sequence() at step 2.
#
# Returns: (results_dict, runner_object)
#   results_dict has at least a "failed" key (count of failed tests).
#   If results["failed"] == 0, all tests passed.
#
# Why separate from boot_sequence()?
#   So that boot_sequence() can handle the LED/switch UI without being
#   cluttered by hardware instantiation logic.
#
# What happens if sensors are not connected?
#   Depending on the driver, .init() may raise an exception (caught and
#   logged by SelfTestRunner) or return a failure TestResult. The POST
#   will mark those tests as failed, and the red LED will blink.
#   If a sensor is missing, tests will fail; the robot will NOT start.
#   If a sensor is plugged in but misconfigured (e.g. wrong I2C address),
#   the test will also fail — this is intentional safety.
def power_on_self_test():
    logger.info("=" * 50)
    logger.info(f"  {BOOT_VERSION}")
    logger.info("  Power-On Self-Test: ALL MODULES")
    logger.info("=" * 50)

    # Load configuration from config/pi_config.yaml.
    # ConfigManager is a singleton, but we instantiate it here to ensure
    # config is loaded before any driver reads from it.
    config = ConfigManager()
    config.load()

    runner = SelfTestRunner()

    # Import drivers INSIDE the function so they are not loaded at module
    # import time (only when POST actually runs).
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

    # --- Instantiate each module --------------------------------------------
    # Camera – no config params needed for POST defaults.
    camera = PiCamera()

    # ToF sensors – xshut_pin AND address must match wiring and
    # pi_config.yaml. Each sensor needs a unique address (0x30/0x31/0x32)
    # or they collide on the I2C bus and every I2C sensor fails.
    tof_left = VL53L0X(
        "left",
        bus=config.get("sensors", "vl53l0x_left", "i2c_bus", default=1),
        address=config.get("sensors", "vl53l0x_left", "address", default=0x30),
        xshut_pin=config.get("sensors", "vl53l0x_left", "xshut_pin", default=None),
    )
    tof_right = VL53L0X(
        "right",
        bus=config.get("sensors", "vl53l0x_right", "i2c_bus", default=1),
        address=config.get("sensors", "vl53l0x_right", "address", default=0x31),
        xshut_pin=config.get("sensors", "vl53l0x_right", "xshut_pin", default=None),
    )
    tof_front = VL53L1X(
        "front",
        bus=config.get("sensors", "vl53l1x_front", "i2c_bus", default=1),
        address=config.get("sensors", "vl53l1x_front", "address", default=0x32),
        xshut_pin=config.get("sensors", "vl53l1x_front", "xshut_pin", default=None),
    )

    # IMU and magnetometer – addresses from config.
    imu = MPU6050(
        bus=config.get("sensors", "mpu6050", "i2c_bus", default=1),
        address=config.get("sensors", "mpu6050", "address", default=0x68),
        accel_range=config.get("sensors", "mpu6050", "accel_range", default=4),
        gyro_range=config.get("sensors", "mpu6050", "gyro_range", default=500),
    )
    mag = QMC5883L(
        bus=config.get("sensors", "qmc5883l", "i2c_bus", default=1),
        address=config.get("sensors", "qmc5883l", "address", default=0x0D),
    )

    # Fusion – UKF and complementary filter.
    # dt=0.01 is the prediction step period; must match the fusion loop rate
    # when used in main.py. For POST, we just check that they initialize.
    ukf = RobotUKF(dt=0.01)
    comp_filter = ComplementaryFilter()

    # Control modules – Stanley + PID controllers.
    stanley = StanleyController()
    servo_pid = ServoPID()
    motor_pid = MotorPID()
    kinematics = KinematicModel()

    # Communications – UART link to Arduino.
    uart = UARTCommunicator()

    # --- Register tests with the runner ------------------------------------
    # Each register_* function adds test cases to the runner for that module.
    register_camera_tests(runner, camera)
    register_tof_tests(runner, {
        "left": tof_left, "right": tof_right, "front": tof_front
    })
    register_imu_tests(runner, imu)
    register_mag_tests(runner, mag)
    register_fusion_tests(runner, ukf, comp_filter)
    register_control_tests(runner, stanley, servo_pid, motor_pid, kinematics)
    register_comm_tests(runner, uart)

    # Run all registered tests.
    results = runner.run_all()

    return results, runner


# =============================================================================
# boot_sequence()
# =============================================================================
# The main boot logic:
#   1. Load config
#   2. Instantiate StatusLED (green + red GPIO pins) and StartSwitch
#   3. Blink pattern to indicate boot start
#   4. Run power_on_self_test()
#   5. If all passed: green LED steady, wait for switch press, return True
#   6. If any failed: red LED blink, log errors, return False
#
# Called by: main() (the script entry point).
#
# GPIO pin configuration (from config/pi_config.yaml):
#   hardware.leds.green_pin – GPIO pin for green LED (default 23)
#   hardware.leds.red_pin   – GPIO pin for red LED   (default 24)
#   hardware.switch.pin     – GPIO pin for start switch (default 25)
# If the wiring changes, these pins must be updated in pi_config.yaml or
# the defaults will be wrong and LEDs/switch won't work.
def boot_sequence():
    # Load configuration. We load again (ConfigManager is a singleton, so
    # this reuses the same instance loaded by power_on_self_test()).
    config = ConfigManager()
    config.load()

    # StatusLED controls two LEDs (green + red) on the specified GPIO pins.
    # The robot uses these to communicate boot status to the human operator.
    led = StatusLED(
        green_pin=config.get("hardware", "leds", "green_pin", default=23),
        red_pin=config.get("hardware", "leds", "red_pin", default=24),
    )
    # StartSwitch is a physical push button. wait_for_press() blocks until
    # the button is pressed (or timeout, if set).
    switch = StartSwitch(
        pin=config.get("hardware", "switch", "pin", default=25),
    )

    # --- Flash all LEDs once (visual confirmation of power-on) --------------
    # interval=0.15 means 150 ms per color. The pattern ["green", "red", "off"]
    # gives a quick green flash, then red, then off.
    led.blink_pattern(["green", "red", "off"], interval=0.15)

    # --- Run the comprehensive self-test ------------------------------------
    results, runner = power_on_self_test()

    # --- Branch on test results ---------------------------------------------
    if results["failed"] == 0:
        # ALL TESTS PASSED
        led.success_sequence()  # e.g., green LED steady on
        logger.info("")
        logger.info("=" * 50)
        logger.info("  ALL TESTS PASSED  ")
        logger.info("  Green LED ON - Press switch to start race")
        logger.info("=" * 50)

        # Wait indefinitely for the physical start switch.
        # timeout=None means block forever until pressed.
        # If a timeout is set (e.g. 30 s) and it expires, the robot could
        # auto-start or shut down depending on design — currently None.
        pressed = switch.wait_for_press(timeout=None)
        if pressed:
            logger.info("Starting race logic...")
            led.stop_blink()
            led.set("blue")  # Blue LED = race mode active
            return True
    else:
        # ONE OR MORE TESTS FAILED
        led.error_sequence()  # e.g., red LED blinking
        logger.info("")
        logger.info("=" * 50)
        logger.info("  SELF-TEST FAILED  ")
        logger.info(f"  {results['failed']} test(s) failed")
        logger.info("  Check logs for details")
        logger.info("=" * 50)
        return False


# =============================================================================
# main()
# =============================================================================
# Top-level entry point when boot.py is run directly.
#   python pi/boot.py
#
# Flow:
#   1. Initialize the Logger (singleton) with name "WRO_BOOT", level INFO,
#      log directory "logs".
#   2. Run boot_sequence().
#   3. If boot_sequence() returns True, launch the main race program
#      (pi.main.main) via asyncio.run().
#   4. If boot_sequence() returns False, log a critical error and exit(1).
#   5. Catch all exceptions to prevent silent crashes.
#
# The race program (pi.main.main) replaces the boot process entirely —
# the boot logger instance remains, but main.py uses it via the same
# global `log` object (Logger is a singleton).
def main():
    # Initialize the singleton logger. This creates both a console handler
    # and a rotating file handler. Level can be changed to DEBUG for more
    # verbose boot logging.
    logger.init(name="WRO_BOOT", level="INFO", log_dir="logs")
    logger.info("WRO 4WS Booting...")

    try:
        success = boot_sequence()
        if success:
            logger.info("Launching main race program...")
            # Import and run the race-mode entry point.
            # This import is deferred until after POST passes so that
            # the large module tree is only loaded when needed.
            from pi.main import main as race_main
            import asyncio
            asyncio.run(race_main())
        else:
            logger.critical("Boot aborted due to test failures")
            sys.exit(1)  # Non-zero exit code signals failure to launcher
    except KeyboardInterrupt:
        logger.info("Boot interrupted")
    except Exception as e:
        logger.critical(f"Boot fatal: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


# =============================================================================
# Script Entry Point
# =============================================================================
# When executed directly (python pi/boot.py), call main().
if __name__ == "__main__":
    main()
