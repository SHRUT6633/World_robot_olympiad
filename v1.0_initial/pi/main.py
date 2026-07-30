import asyncio
import time
import numpy as np
from system.manager import SystemManager
from system.logger import log
from system.config_manager import ConfigManager
from sensors.camera.camera_driver import PiCamera
from sensors.camera.calibration import CameraCalibration
from sensors.camera.pipeline import CameraPipeline
from sensors.tof.vl53l0x import VL53L0X
from sensors.tof.vl53l1x import VL53L1X
from sensors.imu.mpu6050 import MPU6050
from sensors.magnetometer.qmc5883l import QMC5883L
from fusion.ukf import RobotUKF
from fusion.complementary import ComplementaryFilter
from fusion.adaptive_noise import AdaptiveNoiseEstimator
from fusion.mahalanobis import MahalanobisOutlierRejector
from perception.lane_detection import LaneDetector
from perception.wall_detection import WallDetector
from perception.free_space import FreeSpaceDetector
from localization.robot_localization import RobotLocalization
from mission.state_machine import StateMachine, RobotState
from mission.lap_counter import LapCounter
from planning.global_planner import GlobalPlanner
from trajectory.cubic_splines import CubicSplineTrajectory
from trajectory.velocity_profile import VelocityProfiler
from dynamics.kinematic_model import KinematicModel
from control.stanley import StanleyController
from control.servo_pid import ServoPID
from control.motor_pid import MotorPID
from comm.uart import UARTCommunicator


async def main():
    mgr = SystemManager()
    config = mgr.config

    camera = PiCamera(
        device=config.get("sensors", "camera", "device", default=0),
        width=config.get("sensors", "camera", "width", default=640),
        height=config.get("sensors", "camera", "height", default=480),
        fps=config.get("sensors", "camera", "fps", default=60),
    )
    tof_left = VL53L0X("VL53L0X_Left")
    tof_right = VL53L0X("VL53L0X_Right")
    tof_front = VL53L1X("VL53L1X_Front")
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

    state_machine = StateMachine()
    lap_counter = LapCounter(total_laps=2)
    global_planner = GlobalPlanner()
    global_planner.plan_lap(track_width=3.0, track_length=5.0)
    spline = CubicSplineTrajectory()
    vel_profiler = VelocityProfiler()
    kinematics = KinematicModel(wheelbase=0.26)
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
        while True:
            camera_data = camera.read()
            tof_l = tof_left.read()
            tof_r = tof_right.read()
            tof_f = tof_front.read()
            imu_data = imu.read()
            mag_data = mag.read()
            mgr.health.heartbeat("sensors")
            await asyncio.sleep(0.01)

    async def fusion_task():
        while True:
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
                    ukf.ukf.y[0] if hasattr(ukf.ukf, "y") else np.zeros(6),
                    ukf.state,
                )
                state = ukf.state
                localization.pose.update_absolute(state[0], state[1], state[2])
            mgr.health.heartbeat("fusion")
            await asyncio.sleep(0.01)

    async def perception_task():
        while True:
            frame = camera.frame
            if frame is not None:
                lanes = lane_detector.detect(frame)
                free = free_space.detect(frame)
            mgr.health.heartbeat("perception")
            await asyncio.sleep(0.02)

    async def planning_task():
        while True:
            pose = localization.to_dict()
            target = global_planner.get_target(0)
            mgr.health.heartbeat("planning")
            await asyncio.sleep(0.05)

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
            await asyncio.sleep(0.01)

    async def comm_task():
        while True:
            pkt = uart.read()
            mgr.health.heartbeat("comm")
            await asyncio.sleep(0.005)

    async def health_task():
        while True:
            results = mgr.health.check_all()
            dead = [k for k, v in results.items() if not v]
            if dead:
                log.warn(f"Dead components: {dead}")
            await asyncio.sleep(0.5)

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
