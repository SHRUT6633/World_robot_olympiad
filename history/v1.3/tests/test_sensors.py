import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi.sensors.imu.mpu6050 import MPU6050
from pi.sensors.magnetometer.qmc5883l import QMC5883L
from pi.sensors.tof.vl53l0x import VL53L0X
from pi.sensors.tof.vl53l1x import VL53L1X


class TestSensors:
    def test_mpu6050_init(self):
        imu = MPU6050()
        imu.init()
        assert imu is not None

    def test_mpu6050_read(self):
        imu = MPU6050()
        imu.init()
        data = imu.read()
        assert data is not None
        assert "accel" in data
        assert "gyro" in data
        assert len(data["accel"]) == 3
        assert len(data["gyro"]) == 3

    def test_qmc5883l_init(self):
        mag = QMC5883L()
        mag.init()
        assert mag is not None

    def test_qmc5883l_read(self):
        mag = QMC5883L()
        mag.init()
        data = mag.read()
        assert data is not None

    def test_vl53l0x_read(self):
        sensor = VL53L0X("test_left")
        sensor.init()
        data = sensor.read()
        assert data is not None
        assert data > 0

    def test_vl53l1x_read(self):
        sensor = VL53L1X("test_front")
        sensor.init()
        data = sensor.read()
        assert data is not None
        assert data > 0
