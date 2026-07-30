import pytest
import math
import time
from reverse_logic import (
    ReverseLogic, ReverseState, ReverseResult, Pose,
)


class TestReverseLogic:
    @pytest.fixture
    def logic(self):
        return ReverseLogic(
            stuck_timeout=0.1,
            progress_threshold=0.02,
            max_reverse_distance=0.20,
            rear_safety_margin=0.15,
        )

    def test_initial_idle(self, logic):
        assert not logic.is_active()
        assert logic.get_stuck_count() == 0

    def test_moving_robot_not_stuck(self, logic):
        pose = Pose(0, 0, 0)
        assert not logic.check_stuck(pose)

        for i in range(1, 10):
            pose = Pose(0.05 * i, 0, 0)
            assert not logic.check_stuck(pose)

    def test_stationary_robot_detected_stuck(self, logic):
        pose = Pose(0, 0, 0)
        logic.check_stuck(pose)

        pose2 = Pose(0.001, 0, 0)
        result = logic.check_stuck(pose2)
        time.sleep(0.15)
        result = logic.check_stuck(pose2)
        assert result is True

    def test_stuck_count_increments(self, logic):
        pose = Pose(0, 0, 0)
        logic.check_stuck(pose)
        time.sleep(0.15)
        assert logic.check_stuck(pose) is True
        assert logic.get_stuck_count() == 1

    def test_start_reverse(self, logic):
        logic.start_reverse(Pose(0, 0, 0))
        assert logic.is_reversing()
        assert logic.get_reverse_distance() == 0.0

    def test_reverse_distance_limited(self, logic):
        logic.start_reverse(Pose(0, 0, 0))
        result = logic.execute_reverse(Pose(0.05, 0, 0), rear_distance=0.5)
        assert result == ReverseResult.CONTINUE_REVERSE

        result = logic.execute_reverse(Pose(0.19, 0, 0), rear_distance=0.5)
        assert result == ReverseResult.CONTINUE_REVERSE

        result = logic.execute_reverse(Pose(0.21, 0, 0), rear_distance=0.5)
        assert result == ReverseResult.STOP_REVERSE

    def test_rear_sensor_aborts_reverse(self, logic):
        logic.start_reverse(Pose(0, 0, 0))
        result = logic.execute_reverse(Pose(0.05, 0, 0), rear_distance=0.1)
        assert result == ReverseResult.ABORT_REVERSE

    def test_recovery_turn_alternates(self, logic):
        logic._stuck_count = 0
        turn1 = logic.get_recovery_turn()
        logic._stuck_count = 1
        turn2 = logic.get_recovery_turn()
        assert turn1 > 0
        assert turn2 < 0
        assert abs(turn1) == abs(turn2)

    def test_reverse_complete_transitions_to_recovering(self, logic):
        logic.start_reverse(Pose(0, 0, 0))
        logic.execute_reverse(Pose(0.21, 0, 0), rear_distance=0.5)
        logic.on_reverse_complete()
        assert logic.is_recovering()
        assert not logic.is_reversing()

    def test_recovery_complete_returns_to_idle(self, logic):
        logic.start_reverse(Pose(0, 0, 0))
        logic.execute_reverse(Pose(0.21, 0, 0), rear_distance=0.5)
        logic.on_reverse_complete()
        logic.on_recovery_complete()
        assert not logic.is_active()
        assert logic.reverse_state == ReverseState.IDLE

    def test_reset(self, logic):
        logic.start_reverse(Pose(0, 0, 0))
        logic._stuck_count = 3
        logic.reset()
        assert not logic.is_active()
        assert logic.get_stuck_count() == 0

    def test_small_progress_resets_stuck_timer(self, logic):
        pose = Pose(0, 0, 0)
        logic.check_stuck(pose)
        time.sleep(0.05)
        pose2 = Pose(0.03, 0, 0)
        logic.check_stuck(pose2)
        time.sleep(0.05)
        result = logic.check_stuck(pose2)
        assert result is False

    def test_movement_resets_before_timeout(self, logic):
        pose = Pose(0, 0, 0)
        logic.check_stuck(pose)
        time.sleep(0.08)
        pose2 = Pose(0.01, 0, 0)
        assert not logic.check_stuck(pose2)
        time.sleep(0.08)
        assert logic.check_stuck(pose2) is False
