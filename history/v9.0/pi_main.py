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
#      (init / close) and health heartbeats are managed centrally.
#   4. Defines one async coroutine for each subsystem (sensor reading,
#      sensor fusion, perception, planning, control, comms, health).
#   5. Adds each coroutine to the TaskScheduler with a target frequency (Hz)
#      and a priority (higher = runs first within a scheduler tick).
#   6. Calls mgr.init_all() which calls .init() on every registered component.
#   7. Calls mgr.run() which starts the scheduler loop (infinite until
#      KeyboardInterrupt or SIGINT/SIGTERM).
# =============================================================================

import asyncio
import sys
from pathlib import Path

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
from pi.perception.pillar_detector import PillarDetector
from pi.perception.pillar_tracker import PillarTracker
from pi.perception.parking_detector import ParkingDetector
from pi.localization.robot_localization import RobotLocalization
from pi.localization.track_map import TrackMap
from pi.mission.state_machine import StateMachine, RobotState
from pi.mission.lap_counter import LapCounter
from pi.planning.global_planner import GlobalPlanner
from pi.trajectory.cubic_splines import CubicSplineTrajectory
from pi.trajectory.velocity_profile import VelocityProfiler
from pi.dynamics.kinematic_model import KinematicModel
from pi.dynamics.steering_modes import SteeringMode
from pi.control.stanley import StanleyController
from pi.control.servo_pid import ServoPID
from pi.control.motor_pid import MotorPID
from pi.comm.uart import UARTCommunicator


async def main():
    # SystemManager owns TaskScheduler, HealthMonitor, PerformanceMonitor.
    mgr = SystemManager()
    config = mgr.config

    # Surprise rules loaded from config/surprise_rules.yaml
    sr = config.get("surprise_rules", {})
    pillar_logic = sr.get("pillar_logic", "NORMAL")
    drive_direction = sr.get("drive_direction", "BEST_FIT")
    steering_mode_str = sr.get("steering_mode", "SAME_PHASE")
    obstacle_pass_side = sr.get("obstacle_pass_side", "DYNAMIC")
    parking_mode = sr.get("parking_mode", "STANDARD")
    stop_and_go = sr.get("stop_and_go", "DISABLED")
    narrow_track = sr.get("narrow_track", "DISABLED")
    max_speed_override = sr.get("max_speed_ms", 2.0)
    steering_mode = getattr(SteeringMode, steering_mode_str, SteeringMode.SAME_PHASE)
    log.info(f"Surprise config: pillar={pillar_logic} dir={drive_direction} "
             f"steer={steering_mode_str} pass={obstacle_pass_side} "
             f"park={parking_mode} stopgo={stop_and_go} narrow={narrow_track}")

    # Camera — primary forward-facing sensor
    camera = PiCamera(
        device=config.get("sensors", "camera", "device", default=0),
        width=config.get("sensors", "camera", "width", default=640),
        height=config.get("sensors", "camera", "height", default=480),
        fps=config.get("sensors", "camera", "fps", default=60),
    )

    # Time-of-Flight distance sensors: left, right, front
    tof_left = VL53L0X("VL53L0X_Left",
        xshut_pin=config.get("sensors", "vl53l0x_left", "xshut_pin", default=None))
    tof_right = VL53L0X("VL53L0X_Right",
        xshut_pin=config.get("sensors", "vl53l0x_right", "xshut_pin", default=None))
    tof_front = VL53L1X("VL53L1X_Front",
        xshut_pin=config.get("sensors", "vl53l1x_front", "xshut_pin", default=None))

    # IMU and magnetometer
    imu = MPU6050()
    mag = QMC5883L()

    # Sensor fusion modules
    ukf = RobotUKF(dt=0.01)
    comp_filter = ComplementaryFilter()
    adaptive_noise = AdaptiveNoiseEstimator()
    outlier_rejector = MahalanobisOutlierRejector()
    localization = RobotLocalization()
    localization.attach_filter(ukf)

    # Perception modules
    lane_detector = LaneDetector()
    wall_detector = WallDetector()
    free_space = FreeSpaceDetector()
    pillar_detector = PillarDetector(config=sr.get("colour_thresholds", {}))
    pillar_tracker = PillarTracker(pillar_logic=pillar_logic)
    robot_len_mm = config.get("surprise_rules", "parking", "robot_length_mm", default=200)
    parking_detector = ParkingDetector(robot_length_mm=robot_len_mm)
    track_map = TrackMap(track_width_mm=1000)

    # Mission & planning
    state_machine = StateMachine()
    lap_counter = LapCounter(total_laps=3)
    global_planner = GlobalPlanner()
    global_planner.plan_lap(track_width=3.0, track_length=5.0)

    # Trajectory & dynamics
    spline = CubicSplineTrajectory()
    vel_profiler = VelocityProfiler()
    kinematics = KinematicModel(wheelbase=0.26, steering_mode=steering_mode)

    # Control
    stanley = StanleyController()
    servo_pid = ServoPID()
    motor_pid = MotorPID()

    # Communications
    uart = UARTCommunicator()

    # Component registration
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

    # Sensor task (100 Hz)
    async def sensor_task():
        camera_data = camera.read()
        tof_l = tof_left.read()
        tof_r = tof_right.read()
        tof_f = tof_front.read()
        imu_data = imu.read()
        mag_data = mag.read()
        mgr.health.heartbeat("sensors")

    # Fusion task (100 Hz)
    async def fusion_task():
        imu_data = imu.read()
        mag_data = mag.read()
        if imu_data:
            accel, gyro = imu_data["accel"], imu_data["gyro"]
            heading = mag.heading(mag_data, accel) if mag_data is not None else None
            pitch, roll, yaw = comp_filter.update(accel, gyro, heading)
            z = np.array([0.0, 0.0, yaw, 0.0, 0.0, gyro[2]])
            ukf.predict()
            ukf.update(z)
            adaptive_noise.update(
                ukf.ukf.y[0] if hasattr(ukf.ukf, "y") else np.zeros(6), ukf.state)
            state = ukf.state
            localization.pose.update_absolute(state[0], state[1], state[2])
        mgr.health.heartbeat("fusion")

    # Perception task (50 Hz)
    async def perception_task():
        frame = camera.frame
        if frame is not None:
            lanes = lane_detector.detect(frame)
            free = free_space.detect(frame)
            pillar_dets = pillar_detector.detect(frame)
            pillar_tracker.update(pillar_dets)
            pink_dets = pillar_dets.get("pink", [])
            parking_detector.update(pink_dets, tof_left.read(), tof_right.read())
        mgr.health.heartbeat("perception")

    # Planning task (20 Hz)
    async def planning_task():
        pose = localization.to_dict()
        target = global_planner.get_target(0)
        mgr.health.heartbeat("planning")

    # Control task (100 Hz)
    async def control_task():
        pose = localization.to_dict()
        target = global_planner.get_target(0)
        if target is not None:
            target_heading = np.arctan2(
                target[1] - pose["y"], target[0] - pose["x"])
            steering = stanley.compute(
                pose["x"], pose["y"], pose["heading"],
                target[0], target[1], target_heading, pose["v"])
            motor_speed = motor_pid.compute_speed(1.0, pose["v"])
            servo_angle = servo_pid.compute_angle(steering, 0.0)
            uart.send_steering(servo_angle, motor_speed)
        mgr.health.heartbeat("control")

    # Communications task (200 Hz)
    async def comm_task():
        pkt = uart.read()
        mgr.health.heartbeat("comm")

    # Health monitor task (2 Hz)
    async def health_task():
        results = mgr.health.check_all()
        dead = [k for k, v in results.items() if not v]
        if dead:
            log.warn(f"Dead components: {dead}")

    # Schedule all tasks with Hz and priority
    mgr.scheduler.add("sensors", sensor_task, hz=100, priority=10)
    mgr.scheduler.add("fusion", fusion_task, hz=100, priority=9)
    mgr.scheduler.add("perception", perception_task, hz=50, priority=8)
    mgr.scheduler.add("planning", planning_task, hz=20, priority=7)
    mgr.scheduler.add("control", control_task, hz=100, priority=10)
    mgr.scheduler.add("comm", comm_task, hz=200, priority=9)
    mgr.scheduler.add("health", health_task, hz=2, priority=0)

    await mgr.init_all()

    log.info("=" * 50)
    log.info("WRO 4WS Robot - READY")
    log.info("=" * 50)

    try:
        await mgr.run()
    except KeyboardInterrupt:
        await mgr.stop()


if __name__ == "__main__":
    asyncio.run(main())
