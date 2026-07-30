import pytest
import math
import random
from lap_counter import LapCounter, Pose, normalize_angle


@pytest.fixture
def start_line():
    return Pose(0.0, 0.0)


@pytest.fixture
def counter(start_line):
    return LapCounter(
        line_position=start_line,
        zone_radius=0.30,
        clear_distance=0.50,
        expected_heading=0.0,
        max_laps=3,
    )


class TestLapCounter:
    def test_initial_state(self, counter):
        assert counter.get_lap_count() == 0
        assert not counter.is_finished()
        assert counter.get_remaining_laps() == 3

    def test_single_crossing_counts_one_lap(self, counter):
        pose = Pose(0.1, 0.0, 0.0)
        result = counter.process_pose(pose)
        assert result is True
        assert counter.get_lap_count() == 1

    def test_double_count_prevented_hysteresis(self, counter):
        pose = Pose(0.1, 0.0, 0.0)
        counter.process_pose(pose)

        for _ in range(10):
            pose = Pose(
                0.1 + random.uniform(-0.05, 0.05),
                random.uniform(-0.02, 0.02),
                random.uniform(-0.05, 0.05),
            )
            counter.process_pose(pose)

        assert counter.get_lap_count() == 1

    def test_clear_zone_before_next_lap(self, counter):
        counter.process_pose(Pose(0.1, 0.0, 0.0))
        assert counter.get_lap_count() == 1

        counter.process_pose(Pose(0.1, 0.0, 0.0))
        assert counter.get_lap_count() == 1

        counter.process_pose(Pose(0.6, 0.0, 0.0))
        assert counter.get_lap_count() == 1

        counter.process_pose(Pose(0.1, 0.0, 0.0))
        assert counter.get_lap_count() == 2

    def test_wrong_heading_rejected(self, counter):
        pose = Pose(0.1, 0.0, math.radians(90))
        result = counter.process_pose(pose)
        assert result is False
        assert counter.get_lap_count() == 0

    def test_max_laps_three(self, counter):
        for _ in range(3):
            counter.process_pose(Pose(0.1, 0.0, 0.0))
            counter.process_pose(Pose(0.6, 0.0, 0.0))

        assert counter.is_finished()
        assert counter.get_remaining_laps() == 0

    def test_lap_times_recorded(self, counter):
        counter.process_pose(Pose(0.1, 0.0, 0.0))
        counter.process_pose(Pose(0.6, 0.0, 0.0))
        import time
        time.sleep(0.01)
        counter.process_pose(Pose(0.1, 0.0, 0.0))

        assert len(counter.lap_times) == 1
        assert counter.get_last_lap_time() > 0.0
        assert counter.get_average_lap_time() > 0.0

    def test_reset_clears_state(self, counter):
        counter.process_pose(Pose(0.1, 0.0, 0.0))
        counter.process_pose(Pose(0.6, 0.0, 0.0))
        counter.process_pose(Pose(0.1, 0.0, 0.0))
        counter.reset()
        assert counter.get_lap_count() == 0
        assert counter.armed is True
        assert len(counter.lap_times) == 0

    def test_noisy_pose_does_not_double_count(self, counter):
        counter.process_pose(Pose(0.1, 0.0, 0.0))

        random.seed(42)
        for _ in range(100):
            noisy = Pose(
                0.1 + random.gauss(0, 0.02),
                0.0 + random.gauss(0, 0.02),
                random.gauss(0, 0.01),
            )
            counter.process_pose(noisy)

        assert counter.get_lap_count() == 1

    def test_just_outside_zone_not_counted(self, counter):
        pose = Pose(0.31, 0.0, 0.0)
        result = counter.process_pose(pose)
        assert result is False
        assert counter.get_lap_count() == 0
