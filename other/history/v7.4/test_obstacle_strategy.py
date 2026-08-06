import pytest
import math
from obstacle_strategy import (
    ObstacleStrategy, ObstacleManeuver, SensorData,
    PASS_LEFT, PASS_RIGHT, PASS_DYNAMIC, ManeuverState,
)


class TestObstacleStrategy:
    @pytest.fixture
    def strategy(self):
        config = {
            "pass_side": PASS_DYNAMIC,
            "default_pass_side": PASS_LEFT,
            "min_pass_width": 0.25,
            "lock_on_entry": True,
        }
        return ObstacleStrategy(config)

    def test_no_obstacle_no_maneuver(self, strategy):
        data = SensorData(front=0.8, front_left=0.5, front_right=0.5)
        result = strategy.update(data, None)
        assert result is None
        assert not strategy.is_maneuvering()

    def test_obstacle_triggers_maneuver(self, strategy):
        data = SensorData(front=0.2, front_left=0.4, front_right=0.4)
        result = strategy.update(data, None)
        assert result is not None
        assert strategy.is_maneuvering()

    def test_dynamic_chooses_wider_side(self, strategy):
        data = SensorData(
            front=0.2, front_left=0.6, front_right=0.3,
        )
        result = strategy.update(data, None)
        assert result.get_pass_side() == PASS_LEFT

    def test_dynamic_chooses_right_when_wider(self, strategy):
        data = SensorData(
            front=0.2, front_left=0.3, front_right=0.6,
        )
        result = strategy.update(data, None)
        assert result.get_pass_side() == PASS_RIGHT

    def test_strategy_locked_during_maneuver(self, strategy):
        data1 = SensorData(front=0.2, front_left=0.4, front_right=0.6)
        result1 = strategy.update(data1, None)
        assert result1.get_pass_side() == PASS_RIGHT

        data2 = SensorData(
            front=0.6, front_left=0.7, front_right=0.2,
        )
        result2 = strategy.update(data2, None)
        assert result2 is not None
        assert result2.get_pass_side() == PASS_RIGHT

    def test_always_left_config(self):
        config = {"pass_side": PASS_LEFT}
        strategy = ObstacleStrategy(config)
        data = SensorData(front=0.2, front_left=0.3, front_right=0.6)
        result = strategy.update(data, None)
        assert result.get_pass_side() == PASS_LEFT

    def test_always_right_config(self):
        config = {"pass_side": PASS_RIGHT}
        strategy = ObstacleStrategy(config)
        data = SensorData(front=0.2, front_left=0.6, front_right=0.3)
        result = strategy.update(data, None)
        assert result.get_pass_side() == PASS_RIGHT

    def test_maneuver_clears_when_obstacle_passed(self, strategy):
        data1 = SensorData(front=0.2, front_left=0.4, front_right=0.4)
        strategy.update(data1, None)
        assert strategy.is_maneuvering()
        assert strategy.get_maneuver_state() == ManeuverState.ENTERING

        data2 = SensorData(front=0.3, front_left=0.4, front_right=0.4)
        strategy.update(data2, None)
        assert strategy.get_maneuver_state() == ManeuverState.AVOIDING

        data3 = SensorData(front=0.6, front_left=0.6, front_right=0.6)
        strategy.update(data3, None)
        assert strategy.get_maneuver_state() == ManeuverState.CLEARING

        data4 = SensorData(front=0.9, front_left=0.8, front_right=0.8)
        strategy.update(data4, None)
        assert not strategy.is_maneuvering()

    def test_abort_on_infeasible(self):
        config = {
            "pass_side": PASS_LEFT,
            "min_pass_width": 0.25,
            "abort_if_infeasible": True,
        }
        strategy = ObstacleStrategy(config)
        data = SensorData(front=0.2, front_left=0.4, front_right=0.4)
        strategy.update(data, None)

        blocked_data = SensorData(front=0.5, left=0.1, right=0.5)
        strategy.update(blocked_data, None)
        assert not strategy.is_maneuvering()

    def test_reset(self, strategy):
        data = SensorData(front=0.2, front_left=0.4, front_right=0.4)
        strategy.update(data, None)
        assert strategy.is_maneuvering()
        strategy.reset()
        assert not strategy.is_maneuvering()
        assert strategy.current_maneuver is None
