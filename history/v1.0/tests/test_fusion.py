import pytest
import sys
import os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi.fusion.ukf import RobotUKF
from pi.fusion.complementary import ComplementaryFilter
from pi.fusion.mahalanobis import MahalanobisOutlierRejector


class TestFusion:
    def test_ukf_predict(self):
        ukf = RobotUKF(dt=0.01)
        ukf.predict()
        assert ukf.state is not None
        assert len(ukf.state) == 6

    def test_ukf_update(self):
        ukf = RobotUKF(dt=0.01)
        z = np.array([0.1, 0.2, 0.05, 0.5, 0.0, 0.0])
        ukf.predict()
        ukf.update(z)
        assert ukf.state is not None

    def test_complementary(self):
        cf = ComplementaryFilter(alpha=0.98)
        accel = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.01, 0.02, 0.005])
        pitch, roll, yaw = cf.update(accel, gyro)
        assert isinstance(pitch, float)
        assert isinstance(roll, float)

    def test_mahalanobis(self):
        rejector = MahalanobisOutlierRejector(threshold=3.0)
        z = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        mean = np.zeros(6)
        cov = np.eye(6) * 0.1
        mask = rejector.reject_outliers(z, mean, cov)
        assert len(mask) == 6
