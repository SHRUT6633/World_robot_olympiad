import pytest
import time
import math
from race_strategy import RaceStrategy


class TestRaceStrategy:
    @pytest.fixture
    def strategy(self):
        return RaceStrategy(
            min_speed=0.15,
            max_speed=0.35,
            initial_confidence=0.5,
            smoothing_time=0.1,
        )

    def test_initial_confidence(self, strategy):
        assert strategy.get_confidence() == 0.5
        assert strategy.get_confidence_level() == "medium"
        assert not strategy.is_confident()

    def test_initial_speed_mid_range(self, strategy):
        speed = strategy.get_target_speed()
        assert speed == pytest.approx(0.25, abs=0.01)

    def test_lap_completed_boost_confidence(self, strategy):
        strategy.apply_event("lap_completed")
        assert strategy.get_confidence() == pytest.approx(0.65, abs=0.01)
        assert strategy.get_laps_completed() == 1

    def test_lap_bonus_increases_with_each_lap(self, strategy):
        strategy.apply_event("lap_completed")
        c1 = strategy.get_confidence()
        strategy.apply_event("lap_completed")
        c2 = strategy.get_confidence()
        strategy.apply_event("lap_completed")
        c3 = strategy.get_confidence()
        assert c1 < c2 < c3
        assert strategy.get_laps_completed() == 3

    def test_stuck_decreases_confidence(self, strategy):
        strategy.apply_event("stuck_detected")
        assert strategy.get_confidence() == pytest.approx(0.35, abs=0.01)

    def test_emergency_stop_large_penalty(self, strategy):
        strategy.apply_event("emergency_stop")
        assert strategy.get_confidence() == pytest.approx(0.20, abs=0.01)

    def test_confidence_clamped_to_zero(self, strategy):
        strategy.apply_event("emergency_stop")
        strategy.apply_event("emergency_stop")
        assert strategy.get_confidence() == 0.0

    def test_confidence_clamped_to_one(self, strategy):
        for _ in range(10):
            strategy.apply_event("lap_completed")
        assert strategy.get_confidence() == 1.0

    def test_speed_increases_with_confidence(self, strategy):
        strategy.apply_event("lap_completed")
        strategy.apply_event("lap_completed")
        speed = strategy.get_target_speed()
        assert speed > 0.30

    def test_smoothed_speed_approaches_target(self, strategy):
        strategy.apply_event("lap_completed")
        strategy.apply_event("lap_completed")
        target = strategy.get_target_speed()

        for _ in range(50):
            smoothed = strategy.get_smoothed_speed(dt=0.1)
        assert smoothed == pytest.approx(target, abs=0.02)

    def test_low_confidence_slow_speed(self):
        strategy = RaceStrategy(initial_confidence=0.0)
        assert strategy.get_target_speed() == strategy.min_speed
        assert strategy.get_confidence_level() == "low"

    def test_high_confidence_fast_speed(self):
        strategy = RaceStrategy(initial_confidence=1.0)
        assert strategy.get_target_speed() == strategy.max_speed
        assert strategy.get_confidence_level() == "high"
        assert strategy.is_confident()

    def test_event_log_records(self, strategy):
        strategy.apply_event("corner_successful")
        strategy.apply_event("obstacle_avoided")
        log = strategy.get_event_log()
        assert len(log) == 2
        assert log[0]["event"] == "corner_successful"
        assert log[1]["event"] == "obstacle_avoided"

    def test_unknown_event_warns(self, strategy, caplog):
        strategy.apply_event("nonexistent")
        assert "Unknown confidence event" in caplog.text

    def test_reset(self, strategy):
        strategy.apply_event("lap_completed")
        strategy.apply_event("lap_completed")
        strategy.reset()
        assert strategy.get_confidence() == 0.5
        assert strategy.get_laps_completed() == 0
        assert len(strategy.get_event_log()) == 0
