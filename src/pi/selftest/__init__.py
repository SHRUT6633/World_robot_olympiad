# selftest package initialiser.
# Exports the SelfTestRunner and TestResult classes, and imports all
# test-registration functions so that tests_*.py modules can be discovered
# when the package is imported.  Each register function takes a
# SelfTestRunner instance and the relevant hardware abstraction, and adds
# test cases to the runner.

from .runner import SelfTestRunner, TestResult
from .tests_camera import register_camera_tests
from .tests_tof import register_tof_tests, register_tof_cross_test
from .tests_imu import register_imu_tests
from .tests_mag import register_mag_tests
from .tests_fusion import register_fusion_tests
from .tests_control import register_control_tests
from .tests_comm import register_comm_tests
