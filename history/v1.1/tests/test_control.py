import pytest
import sys
import os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi.control.stanley import StanleyController
from pi.control.adaptive_pid import AdaptivePID
from pi.dynamics.kinematic_model import KinematicModel


class TestControl:
    def test_stanley(self):
        ctrl = StanleyController(k=0.5)
        steer = ctrl.compute(0, 0, 0, 1, 0, 0, 1.0)
        assert isinstance(steer, float)
        assert abs(steer) <= ctrl.max_steering

    def test_pid(self):
        pid = AdaptivePID(kp=1.0, ki=0.0, kd=0.0)
        output = pid.compute(1.0)
        assert output == pytest.approx(1.0, rel=0.1)

    def test_kinematics(self):
        model = KinematicModel(wheelbase=0.26)
        x, y, h = model.update(0, 0, 0, 1.0, 0.1, 0.01)
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert h != 0.0

    def test_pid_reset(self):
        pid = AdaptivePID(kp=1.0, ki=1.0, kd=0.0)
        pid.compute(1.0)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._last_error == 0.0
