# =============================================================================
# main.py — WRO 4WS Robot Race Entry Point
# =============================================================================
# This module is the top-level entry point for the robot's race-mode logic.
# It is called either:
#   - From boot.py's main() after the self-test passes and the start switch
#     is pressed, OR
#   - Directly via `python -m pi.main` for development/testing (skipping POST).
#
# What it does:
#   1. Instantiates all hardware drivers (camera, ToF, IMU, magnetometer).
#   2. Instantiates all software modules (filtering, perception, planning,
#      control, communications, health monitoring).
#   3. Registers every component with the SystemManager so lifecycle
#     (init / close) and health heartbeats are managed centrally.
#   4. Defines one async coroutine for each subsystem (sensor reading,
#      sensor fusion, perception, planning, control, comms, health).
#   5. Adds each coroutine to the TaskScheduler with a target frequency (Hz)
#      and a priority (higher = runs first within a scheduler tick).
#   6. Calls mgr.init_all() which calls .init() on every registered component.
#   7. Calls mgr.run() which starts the scheduler loop (infinite until
#      KeyboardInterrupt or SIGINT/SIGTERM).
#
# =============================================================================

import asyncio
import sys
from pathlib import Path

# Add the project root (one level above pi/) to sys.path so that all
# pi.xxx imports resolve correctly regardless of how the script is invoked.
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import numpy as np
from pi.system.manager import SystemManager
from pi.system.logger import log
log.init()
from pi.system.config_manager import ConfigManager
from pi.sensors.camera.camera_driver import PiCamera
from pi.sensors.camera.calibration import CameraCalibration
from pi.sensors.camera.pipeline import CameraPipeline
from pi.sensors.tof.vl53l0x import VL53L0X
from pi.sensors.tof.vl53l1x import VL53L1X
from pi.sensors.imu.mpu6050 import MPU6050
from pi.sensors.magnetometer.qmc5883l import QMC5883L
from pi.fusion.ukf import RobotUKF
from pi.fusion.complementary import ComplementaryFilter
from pi.fusion.adaptive_noise import AdaptiveNoiseEstimator
from pi.fusion.mahalanobis import MahalanobisOutlierRejector
from pi.perception.lane_detection import LaneDetector
from pi.perception.wall_detection import WallDetector
from pi.perception.free_space import FreeSpaceDetector
from pi.localization.robot_localization import RobotLocalization
from pi.mission.state_machine import StateMachine, RobotState
from pi.mission.lap_counter import LapCounter
from pi.planning.global_planner import GlobalPlanner
from pi.trajectory.cubic_splines import CubicSplineTrajectory
from pi.trajectory.velocity_profile import VelocityProfiler
from pi.dynamics.kinematic_model import KinematicModel
from pi.control.stanley import StanleyController
from pi.control.servo_pid import ServoPID
from pi.control.motor_pid import MotorPID
from pi.comm.uart import UARTCommunicator


async def main():
    # =========================================================================
    # SystemManager
    # =========================================================================
    # Singleton orchestrator that owns ConfigManager, TaskScheduler,
    # HealthMonitor, PerformanceMonitor, and the component registry.
    # It calls .init() on every registered component at startup and .close()
    # on shutdown.
    mgr = SystemManager()
    config = mgr.config  # ConfigManager singleton, already loaded

    # =========================================================================
    # Camera (PiCamera)
    # =========================================================================
    # The primary forward-facing camera. Config keys control:
    #   device   – V4L2 device index (0 = /dev/video0); change if multiple cams
    #   width    – capture width in pixels (e.g. 640); smaller = faster, less detail
    #   height   – capture height in pixels (e.g. 480); smaller = faster
    #   fps      – target framerate (e.g. 60); increase = smoother but more CPU
    # Changing these affects perception latency and accuracy.
    camera = PiCamera(
        device=config.get("sensors", "camera", "device", default=0),
        width=config.get("sensors", "camera", "width", default=640),
        height=config.get("sensors", "camera", "height", default=480),
        fps=config.get("sensors", "camera", "fps", default=60),
    )

    # =========================================================================
    # Time-of-Flight Distance Sensors
    # =========================================================================
    # Two VL53L0X (short-range, ~2 m) for left/right lateral distance.
    # One VL53L1X (long-range, ~4 m) for forward distance.
    # xshut_pin: GPIO pin used to selectively enable/disable each sensor
    #   during I2C address assignment. If xshut_pin is None, the driver uses
    #   the default I2C address (0x29) — only works if only ONE ToF is
    #   connected. For multiple ToFs, each must have a unique xshut_pin so
    #   the driver can power-cycle them one at a time to assign distinct
    #   addresses. Wiring must match these pin assignments.
    tof_left = VL53L0X(
        "VL53L0X_Left",
        xshut_pin=config.get("sensors", "vl53l0x_left", "xshut_pin", default=None),
    )
    tof_right = VL53L0X(
        "VL53L0X_Right",
        xshut_pin=config.get("sensors", "vl53l0x_right", "xshut_pin", default=None),
    )
    tof_front = VL53L1X(
        "VL53L1X_Front",
        xshut_pin=config.get("sensors", "vl53l1x_front", "xshut_pin", default=None),
    )

    # =========================================================================
    # IMU (MPU6050) and Magnetometer (QMC5883L)
    # =========================================================================
    # MPU6050: 6-DoF (accel + gyro). Used for pitch/roll/yaw estimation.
    # QMC5883L: 3-axis magnetometer. Provides absolute heading reference.
    # Both communicate over I2C at fixed addresses (0x68 and 0x0D).
    # No config keys required unless addresses differ.
    imu = MPU6050()
    mag = QMC5883L()

    # =========================================================================
    # Sensor Fusion Modules
    # =========================================================================
    #   ukf (RobotUKF)          – Unscented Kalman Filter; fuses IMU + mag +
    #                             odometry into a 6-DoF state estimate.
    #                             dt=0.01 means 100 Hz prediction step.
    #                             If dt does not match the fusion loop rate,
    #                             the filter's prediction covariance grows
    #                             incorrectly. Keep dt == 1/fusion_hz.
    #   comp_filter             – ComplementaryFilter; fuses accelerometer
    #                             (gravity vector) and gyro (integration) to
    #                             get drift-free pitch/roll; magnetometer for
    #                             yaw.
    #   adaptive_noise          – Tunes UKF process noise based on innovation
    #                             residuals so the filter trusts sensors more
    #                             when they are consistent and less when erratic.
    #   outlier_rejector        – Mahalanobis distance check; rejects
    #                             measurement updates that are statistical
    #                             outliers (e.g. from sensor glitches).
    #   localization            – RobotLocalization; top-level wrapper that
    #                             holds Pose objects and can switch between
    #                             filter backends. attach_filter() links the UKF.
    ukf = RobotUKF(dt=0.01)
    comp_filter = ComplementaryFilter()
    adaptive_noise = AdaptiveNoiseEstimator()
    outlier_rejector = MahalanobisOutlierRejector()
    localization = RobotLocalization()
    localization.attach_filter(ukf)

    # =========================================================================
    # Perception Modules
    # =========================================================================
    #   lane_detector  – Finds lane boundaries from the camera frame
    #                     (edge detection, Hough transform, or ML).
    #   wall_detector  – Detects walls using ToF point clouds.
    #   free_space     – Classifies drivable vs. obstacle regions.
    lane_detector = LaneDetector()
    wall_detector = WallDetector()
    free_space = FreeSpaceDetector()

    # =========================================================================
    # Mission & Planning
    # =========================================================================
    #   state_machine  – Finite-state machine (e.g. SEARCHING, TRACKING,
    #                    AVOIDING, FINISHED). Drives top-level behavior.
    #   lap_counter    – Counts laps completed using pose or start/finish
    #                    line crossings. total_laps=2 means the robot stops
    #                    after 2 laps.
    #                     Change to match competition lap count.
    #   global_planner – Generates waypoints around the track.
    #                    plan_lap(track_width, track_length) creates a
    #                    rectangular circuit. Adjust for actual track size.
    state_machine = StateMachine()
    lap_counter = LapCounter(total_laps=2)
    global_planner = GlobalPlanner()
    global_planner.plan_lap(track_width=3.0, track_length=5.0)

    # =========================================================================
    # Trajectory & Dynamics
    # =========================================================================
    #   spline        – CubicSplineTrajectory; interpolates smooth paths
    #                   through waypoints.
    #   vel_profiler  – VelocityProfiler; plans acceleration-limited speed
    #                   profiles along the spline.
    #   kinematics    – KinematicModel; bicycle/4WS model.
    #                   wheelbase=0.26 (meters). Change to match actual
    #                   wheelbase length or steering commands will be wrong.
    spline = CubicSplineTrajectory()
    vel_profiler = VelocityProfiler()
    kinematics = KinematicModel(wheelbase=0.26)

    # =========================================================================
    # Control Modules
    # =========================================================================
    #   stanley   – StanleyController; computes steering angle from
    #               cross-track error and heading error.
    #   servo_pid – ServoPID; closed-loop position control for steering servo.
    #   motor_pid – MotorPID; closed-loop speed control for drive motor.
    stanley = StanleyController()
    servo_pid = ServoPID()
    motor_pid = MotorPID()

    # =========================================================================
    # Communications
    # =========================================================================
    # uart – UARTCommunicator; sends steering + speed commands to the
    #        Arduino (or ESC/servo controller) over serial, and receives
    #        telemetry packets (encoder counts, battery voltage, etc.).
    uart = UARTCommunicator()

    # =========================================================================
    # Component Registration
    # =========================================================================
    # Each registered component gets its .init() and .close() called
    # automatically by SystemManager. The name string is also used by
    # HealthMonitor for heartbeat tracking.
    mgr.register("camera", camera)
    mgr.register("tof_left", tof_left)
    mgr.register("tof_right", tof_right)
    mgr.register("tof_front", tof_front)
    mgr.register("imu", imu)
    mgr.register("mag", mag)
    mgr.register("ukf", ukf)
    mgr.register("localization", localization)
    mgr.register("state_machine", state_machine)
    mgr.register("uart", uart)

    # =========================================================================
    # Sensor Task (100 Hz)
    # =========================================================================
    # Reads all raw sensors every 10 ms. camera.read() returns the latest
    # frame; the ToF and IMU/mag reads return latest measurements.
    # Heartbeat "sensors" tells the HealthMonitor this task is alive.
    # If this task stalls, health check will mark "sensors" as dead.
    async def sensor_task():
        while True:
            camera_data = camera.read()
            tof_l = tof_left.read()
            tof_r = tof_right.read()
            tof_f = tof_front.read()
            imu_data = imu.read()
            mag_data = mag.read()
            mgr.health.heartbeat("sensors")
            await asyncio.sleep(0.01)  # 100 Hz

    # =========================================================================
    # Fusion Task (100 Hz)
    # =========================================================================
    # Runs the complementary filter (pitch/roll/yaw) and the UKF (6-DoF state
    # estimate). Also updates the adaptive noise estimator so the filter can
    # react to changing noise conditions. Finally writes the result into the
    # localization module's pose.
    # Heartbeat: "fusion".
    async def fusion_task():
        while True:
            imu_data = imu.read()
            mag_data = mag.read()
            if imu_data:
                accel, gyro = imu_data["accel"], imu_data["gyro"]
                heading = mag.heading(mag_data, accel) if mag_data is not None else None
                pitch, roll, yaw = comp_filter.update(accel, gyro, heading)
                # Measurement vector z: [x, y, yaw, vx, vy, yaw_rate]
                # x, y, vx, vy are 0 here because we don't have direct
                # position/velocity measurements from IMU alone.
                z = np.array([0.0, 0.0, yaw, 0.0, 0.0, gyro[2]])
                ukf.predict()
                ukf.update(z)
                adaptive_noise.update(
                    ukf.ukf.y[0] if hasattr(ukf.ukf, "y") else np.zeros(6),
                    ukf.state,
                )
                state = ukf.state
                localization.pose.update_absolute(state[0], state[1], state[2])
            mgr.health.heartbeat("fusion")
            await asyncio.sleep(0.01)  # 100 Hz

    # =========================================================================
    # Perception Task (50 Hz)
    # =========================================================================
    # Runs lane detection and free-space detection on the latest camera frame.
    # Runs at 50 Hz (20 ms period) — camera may be 60 FPS but perception
    # is typically heavier, so we run it slower.
    # Heartbeat: "perception".
    async def perception_task():
        while True:
            frame = camera.frame
            if frame is not None:
                lanes = lane_detector.detect(frame)
                free = free_space.detect(frame)
            mgr.health.heartbeat("perception")
            await asyncio.sleep(0.02)  # 50 Hz

    # =========================================================================
    # Planning Task (20 Hz)
    # =========================================================================
    # Reads the current pose from localization and asks the global planner
    # for the next target waypoint. In a full implementation this would also
    # trigger local planning / obstacle avoidance.
    # Heartbeat: "planning".
    async def planning_task():
        while True:
            pose = localization.to_dict()
            target = global_planner.get_target(0)
            mgr.health.heartbeat("planning")
            await asyncio.sleep(0.05)  # 20 Hz

    # =========================================================================
    # Control Task (100 Hz)
    # =========================================================================
    # Computes steering (Stanley) and speed (motor PID) and sends commands
    # over UART to the Arduino/motor controller.
    #   target_heading – direction from current position to target waypoint
    #   steering       – Stanley controller output (radians or normalized)
    #   motor_speed    – desired forward speed from motor PID
    #   servo_angle    – servo position from servo PID (tracks steering)
    # Heartbeat: "control".
    async def control_task():
        while True:
            pose = localization.to_dict()
            target = global_planner.get_target(0)
            if target is not None:
                target_heading = np.arctan2(
                    target[1] - pose["y"], target[0] - pose["x"]
                )
                steering = stanley.compute(
                    pose["x"], pose["y"], pose["heading"],
                    target[0], target[1], target_heading, pose["v"],
                )
                motor_speed = motor_pid.compute_speed(1.0, pose["v"])
                servo_angle = servo_pid.compute_angle(steering, 0.0)
                uart.send_steering(servo_angle, motor_speed)
            mgr.health.heartbeat("control")
            await asyncio.sleep(0.01)  # 100 Hz

    # =========================================================================
    # Communications Task (200 Hz)
    # =========================================================================
    # Polls the UART for incoming telemetry packets at 5 ms intervals.
    # Heartbeat: "comm".
    async def comm_task():
        while True:
            pkt = uart.read()
            mgr.health.heartbeat("comm")
            await asyncio.sleep(0.005)  # 200 Hz

    # =========================================================================
    # Health Monitor Task (2 Hz)
    # =========================================================================
    # Checks every registered heartbeat component every 500 ms.
    # If any component's last heartbeat is older than HealthMonitor.timeout_s
    # (default 2.0 s), it is flagged as "dead" and a warning is logged.
    # This is purely diagnostic — it does NOT kill tasks.
    async def health_task():
        while True:
            results = mgr.health.check_all()
            dead = [k for k, v in results.items() if not v]
            if dead:
                log.warn(f"Dead components: {dead}")
            await asyncio.sleep(0.5)  # 2 Hz

    # =========================================================================
    # Schedule Tasks
    # =========================================================================
    #   name       – unique task label, used for stats and health tracking
    #   callback   – async coroutine to run
    #   hz         – target frequency (Hz). The scheduler runs spin_once()
    #                as fast as possible; each task executes when its period
    #                (1/hz) has elapsed since last run.
    #   priority   – within a single tick, tasks are sorted descending by
    #                priority. Higher-priority tasks run first.
    #                Priority 10 = control & sensors (critical real-time),
    #                Priority 0  = health (lowest).
    # Changing Hz values affects timing:
    #   sensors, fusion, control at 100 Hz → 10 ms loop, tight but feasible.
    #   perception at 50 Hz → 20 ms; if detection takes >20 ms, it slips.
    #   comm at 200 Hz → 5 ms; must not block. Health at 2 Hz → lightweight.
    mgr.scheduler.add("sensors", sensor_task, hz=100, priority=10)
    mgr.scheduler.add("fusion", fusion_task, hz=100, priority=9)
    mgr.scheduler.add("perception", perception_task, hz=50, priority=8)
    mgr.scheduler.add("planning", planning_task, hz=20, priority=7)
    mgr.scheduler.add("control", control_task, hz=100, priority=10)
    mgr.scheduler.add("comm", comm_task, hz=200, priority=9)
    mgr.scheduler.add("health", health_task, hz=2, priority=0)

    # =========================================================================
    # Initialize All Components
    # =========================================================================
    # Calls .init() on every registered component. If a component fails,
    # its error is logged but init continues — the system may still start
    # with degraded functionality. PerformanceMonitor is started afterward.
    await mgr.init_all()

    log.info("=" * 50)
    log.info("WRO 4WS Robot - READY")
    log.info("=" * 50)

    # =========================================================================
    # Run (Infinite Loop Until Interrupt)
    # =========================================================================
    # mgr.run() starts the scheduler loop. On KeyboardInterrupt or
    # SIGINT/SIGTERM, it calls mgr.stop() which stops the scheduler and
    # performance monitor, then mgr.shutdown() which calls .close() on
    # each component in reverse registration order.
    try:
        await mgr.run()
    except KeyboardInterrupt:
        await mgr.stop()


# =============================================================================
# Script Entry Point
# =============================================================================
# asyncio.run(main()) starts the async event loop and runs main().
# This is invoked either:
#   - Directly:  python pi/main.py
#   - From boot: asyncio.run(race_main()) in boot.py
if __name__ == "__main__":
    asyncio.run(main())
