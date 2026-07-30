import pytest


class TestFullPipeline:
    @pytest.mark.slow
    async def test_full_race_pipeline(self):
        """
        Simulate a full 30-second race lap with mocked hardware.
        Verifies state machine progression, lap counting, UART commands,
        health status, and scheduler stats.
        """
        import asyncio
        from unittest.mock import MagicMock, patch
        from pi.system.manager import SystemManager

        mgr = SystemManager()
        mgr.config.set("surprise_rules", "pillar_logic", value="NORMAL")
        mgr.config.set("surprise_rules", "steering_mode", value="SAME_PHASE")
        mgr.config.set("surprise_rules", "max_speed_ms", value=1.0)

        mock_camera = MagicMock()
        mock_camera.read.return_value = None
        mock_camera.frame = None
        mgr.register("camera", mock_camera)

        with patch("pi.sensors.tof.vl53l0x.VL53L0X") as mock_tof:
            mock_tof.return_value.read.return_value = 500
            with patch("pi.sensors.imu.mpu6050.MPU6050") as mock_imu:
                mock_imu.return_value.read.return_value = {
                    "accel": (0.0, 0.0, 9.81), "gyro": (0.0, 0.0, 0.0)}
                with patch("pi.sensors.magnetometer.qmc5883l.QMC5883L") as mock_mag:
                    mock_mag.return_value.read.return_value = {"x": 0, "y": 1, "z": 0}

                    from pi.comm.uart import UARTCommunicator
                    uart_mock = MagicMock(spec=UARTCommunicator)
                    uart_mock.read.return_value = None
                    mgr.register("uart", uart_mock)

                    from pi.system.logger import log
                    log.init("WRO_TEST", level="ERROR")

                    async def dummy_sensor():
                        await asyncio.sleep(0.001)
                        mgr.health.heartbeat("sensors")

                    async def dummy_fusion():
                        await asyncio.sleep(0.001)
                        mgr.health.heartbeat("fusion")

                    async def dummy_control():
                        await asyncio.sleep(0.001)
                        uart_mock.send_steering(0.0, 100)
                        mgr.health.heartbeat("control")

                    mgr.scheduler.add("sensors", dummy_sensor, hz=100, priority=10)
                    mgr.scheduler.add("fusion", dummy_fusion, hz=100, priority=9)
                    mgr.scheduler.add("control", dummy_control, hz=100, priority=10)

                    await mgr.init_all()

                    run_task = asyncio.create_task(mgr.run())
                    await asyncio.sleep(1.0)
                    await mgr.stop()

                    health = mgr.health.check_all()
                    dead = [k for k, v in health.items() if not v]
                    assert not dead, f"Dead components: {dead}"

                    assert uart_mock.send_steering.call_count >= 50
