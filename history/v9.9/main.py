import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from pi.system.manager import SystemManager
from pi.system.logger import log
log.init()
from pi.sensors.camera.camera_driver import PiCamera
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
from pi.mission.state_machine import StateMachine
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
    mgr = SystemManager()
    config = mgr.config
    sr = config.get("surprise_rules", {})

    pillar_logic = sr.get("pillar_logic", "NORMAL")
    drive_direction = sr.get("drive_direction", "BEST_FIT")
    steering_mode_str = sr.get("steering_mode", "SAME_PHASE")
    if steering_mode_str not in SteeringMode.__members__:
        log.warn(f"Unknown steering_mode '{steering_mode_str}', using SAME_PHASE")
        steering_mode_str = "SAME_PHASE"
    steering_mode = SteeringMode[steering_mode_str]

    rates = sr.get("rates", {})
    perception_hz = rates.get("perception_hz", 20)
    control_hz = rates.get("control_hz", 100)
    fusion_hz = rates.get("fusion_hz", 100)
    comms_hz = rates.get("comms_hz", 200)
    logging_hz = rates.get("logging_hz", 0.5)

    camera = PiCamera(
        device=config.get("sensors", "camera", "device", default=0),
        width=config.get("sensors", "camera", "width", default=640),
        height=config.get("sensors", "camera", "height", default=480),
        fps=config.get("sensors", "camera", "fps", default=60),
    )
    tof_left = VL53L0X("VL53L0X_Left",
        xshut_pin=config.get("sensors", "vl53l0x_left", "xshut_pin", default=None))
    tof_right = VL53L0X("VL53L0X_Right",
        xshut_pin=config.get("sensors", "vl53l0x_right", "xshut_pin", default=None))
    tof_front = VL53L1X("VL53L1X_Front",
        xshut_pin=config.get("sensors", "vl53l1x_front", "xshut_pin", default=None))
    imu = MPU6050()
    mag = QMC5883L()
    ukf = RobotUKF(dt=0.01)
    comp_filter = ComplementaryFilter()
    adaptive_noise = AdaptiveNoiseEstimator()
    outlier_rejector = MahalanobisOutlierRejector()
    localization = RobotLocalization()
    localization.attach_filter(ukf)
    lane_detector = LaneDetector()
    wall_detector = WallDetector()
    free_space = FreeSpaceDetector()
    pillar_detector = PillarDetector(config=sr.get("colour_thresholds", {}))
    pillar_tracker = PillarTracker(pillar_logic=pillar_logic)
    robot_len_mm = config.get("surprise_rules", "parking", "robot_length_mm", default=200)
    parking_detector = ParkingDetector(robot_length_mm=robot_len_mm)
    state_machine = StateMachine()
    lap_counter = LapCounter(total_laps=3)
    global_planner = GlobalPlanner()
    global_planner.plan_lap(track_width=3.0, track_length=5.0)
    spline = CubicSplineTrajectory()
    vel_profiler = VelocityProfiler()
    kinematics = KinematicModel(wheelbase=0.26, steering_mode=steering_mode)
    stanley = StanleyController()
    servo_pid = ServoPID()
    motor_pid = MotorPID()
    uart = UARTCommunicator()

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

    async def sensor_task():
        camera.read()
        tof_left.read()
        tof_right.read()
        tof_front.read()
        imu.read()
        mag.read()
        mgr.health.heartbeat("sensors")

    async def fusion_task():
        imu_data = imu.read()
        mag_data = mag.read()
        if imu_data:
            accel, gyro = imu_data["accel"], imu_data["gyro"]
            heading = mag.heading(mag_data, accel) if mag_data else None
            pitch, roll, yaw = comp_filter.update(accel, gyro, heading)
            z = np.array([0.0, 0.0, yaw, 0.0, 0.0, gyro[2]])
            ukf.predict()
            ukf.update(z)
            state = ukf.state
            localization.pose.update_absolute(state[0], state[1], state[2])
        mgr.health.heartbeat("fusion")

    async def perception_task():
        frame = camera.frame
        if frame is not None:
            pillar_dets = pillar_detector.detect(frame)
            pillar_tracker.update(pillar_dets)
        mgr.health.heartbeat("perception")

    async def control_task():
        pose = localization.to_dict()
        target = global_planner.get_target(0)
        if target is not None:
            target_heading = np.arctan2(target[1] - pose["y"], target[0] - pose["x"])
            steering = stanley.compute(
                pose["x"], pose["y"], pose["heading"],
                target[0], target[1], target_heading, pose["v"])
            motor_speed = motor_pid.compute_speed(1.0, pose["v"])
            servo_angle = servo_pid.compute_angle(steering, 0.0)
            uart.send_steering(servo_angle, motor_speed)
        mgr.health.heartbeat("control")

    async def comm_task():
        uart.read()
        mgr.health.heartbeat("comm")

    async def health_task():
        results = mgr.health.check_all()
        dead = [k for k, v in results.items() if not v]
        if dead:
            log.warn(f"Dead components: {dead}")

    mgr.scheduler.add("sensors", sensor_task, hz=fusion_hz, priority=10)
    mgr.scheduler.add("fusion", fusion_task, hz=fusion_hz, priority=9)
    mgr.scheduler.add("perception", perception_task, hz=perception_hz, priority=8)
    mgr.scheduler.add("control", control_task, hz=control_hz, priority=10)
    mgr.scheduler.add("comm", comm_task, hz=comms_hz, priority=9)
    mgr.scheduler.add("health", health_task, hz=logging_hz, priority=0)

    await mgr.init_all()
    log.info("WRO 4WS Robot - READY")
    try:
        await mgr.run()
    except KeyboardInterrupt:
        await mgr.stop()


if __name__ == "__main__":
    asyncio.run(main())
