import pytest
import math
import time
from direction_detect import (
    DirectionDetector, DirectionState, Pose,
    CW, CCW, normalize_angle,
)


class TestDirectionDetector:
    @pytest.fixture
    def detector(self):
        return DirectionDetector(
            min_detect_distance=0.5,
            yaw_threshold_deg=45,
        )

    def test_initial_state(self, detector):
        assert detector.get_state() == DirectionState.UNKNOWN
        assert detector.get_direction() is None
        assert not detector.is_detected()

    def test_first_pose_starts_detecting(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        assert detector.get_state() == DirectionState.DETECTING

    def test_straight_line_does_not_detect(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        for i in range(1, 20):
            result = detector.process_pose(Pose(0.1 * i, 0, 0))
            assert result is None
        assert not detector.is_detected()

    def test_left_turn_detects_ccw(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        poses = [
            Pose(0.2, 0.0, math.radians(5)),
            Pose(0.4, 0.05, math.radians(15)),
            Pose(0.6, 0.15, math.radians(30)),
            Pose(0.7, 0.3, math.radians(50)),
        ]
        result = None
        for p in poses:
            result = detector.process_pose(p)
        assert result == CCW
        assert detector.is_detected()

    def test_right_turn_detects_cw(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        poses = [
            Pose(0.2, 0.0, math.radians(-5)),
            Pose(0.4, -0.05, math.radians(-15)),
            Pose(0.6, -0.15, math.radians(-30)),
            Pose(0.7, -0.3, math.radians(-50)),
        ]
        result = None
        for p in poses:
            result = detector.process_pose(p)
        assert result == CW
        assert detector.is_detected()

    def test_short_distance_does_not_trigger(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        result = detector.process_pose(Pose(0.1, 0.1, math.radians(50)))
        assert result is None
        assert not detector.is_detected()

    def test_already_detected_returns_immediately(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        poses = [
            Pose(0.2, 0.0, math.radians(5)),
            Pose(0.4, 0.05, math.radians(15)),
            Pose(0.6, 0.15, math.radians(30)),
            Pose(0.7, 0.3, math.radians(50)),
        ]
        for p in poses:
            detector.process_pose(p)
        result = detector.process_pose(Pose(0.9, 0.5, math.radians(60)))
        assert result == CCW

    def test_reset(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        poses = [
            Pose(0.2, 0.0, math.radians(10)),
            Pose(0.4, 0.05, math.radians(30)),
            Pose(0.6, 0.15, math.radians(50)),
        ]
        for p in poses:
            detector.process_pose(p)
        assert detector.is_detected()
        detector.reset()
        assert not detector.is_detected()
        assert detector.get_direction() is None
        assert detector.get_state() == DirectionState.UNKNOWN

    def test_force_direction(self, detector):
        detector.force_direction(CW)
        assert detector.is_detected()
        assert detector.get_direction() == CW

    def test_force_direction_invalid(self, detector):
        with pytest.raises(ValueError):
            detector.force_direction("invalid")

    def test_yaw_buffer_averages_noise(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        result = detector.process_pose(Pose(0.6, 0, math.radians(-50)))
        assert result is None

        result = detector.process_pose(Pose(0.7, 0.1, math.radians(10)))
        result = detector.process_pose(Pose(0.8, 0.2, math.radians(20)))
        result = detector.process_pose(Pose(0.9, 0.35, math.radians(40)))
        result = detector.process_pose(Pose(1.0, 0.5, math.radians(55)))
        assert result == CCW

    def test_max_yaw_change_recorded(self, detector):
        detector.process_pose(Pose(0, 0, 0))
        poses = [
            Pose(0.6, 0.0, math.radians(10)),
            Pose(0.8, 0.1, math.radians(30)),
            Pose(1.0, 0.3, math.radians(60)),
        ]
        for p in poses:
            detector.process_pose(p)
        assert detector.get_max_yaw_change() > math.radians(45)
