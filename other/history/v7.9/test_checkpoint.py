import pytest
import math
import time
from checkpoint import CheckpointManager, Pose, Section, DEFAULT_SECTIONS


class TestCheckpointManager:
    @pytest.fixture
    def mgr(self):
        return CheckpointManager(look_ahead_distance=0.5)

    def test_initial_no_section(self, mgr):
        assert mgr.get_current_section() is None
        assert mgr.get_transition_count() == 0

    def test_first_update_sets_initial_section(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        assert mgr.get_current_section() is not None
        assert mgr.get_current_section_id() == 1

    def test_section_transition_on_look_ahead(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        assert mgr.get_current_section_id() == 1

        mgr.update(Pose(1.1, 0.0))
        assert mgr.get_current_section_id() == 2
        assert mgr.get_transition_count() == 1

    def test_all_sections_traversed_in_order(self, mgr):
        waypoints = []
        for section in DEFAULT_SECTIONS:
            for wp in section.waypoints:
                if wp not in waypoints:
                    waypoints.append(wp)

        mgr.update(Pose(waypoints[0][0], waypoints[0][1]))
        assert mgr.get_current_section_id() == 1

        for i, wp in enumerate(waypoints[1:-1], start=1):
            mgr.update(Pose(wp[0], wp[1]))
            expected_section = min(i + 1, 8)
            assert mgr.get_current_section_id() == expected_section

        last = waypoints[-1]
        mgr.update(Pose(last[0], last[1]))
        assert mgr.get_current_section_id() == 1
        assert mgr.get_lap_count() == 1

    def test_lap_completed_resets_to_section_1(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        for i in range(2, 9):
            section = DEFAULT_SECTIONS[i - 1]
            wp = section.waypoints[-1]
            mgr.update(Pose(wp[0], wp[1]))

        assert mgr.get_lap_count() == 1
        assert mgr.get_current_section_id() == 1

    def test_behavior_from_section(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        assert mgr.get_current_behavior() == "fast"

        mgr.update(Pose(1.1, 0.0))
        assert mgr.get_current_behavior() == "cornering"

    def test_section_history_tracked(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        mgr.update(Pose(1.1, 0.0))
        assert len(mgr.get_section_history()) == 1
        assert mgr.get_section_history()[0] == 2

    def test_is_on_last_section(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        assert not mgr.is_on_last_section()

        for i in range(2, 8):
            section = DEFAULT_SECTIONS[i - 1]
            wp = section.waypoints[-1]
            mgr.update(Pose(wp[0], wp[1]))
        assert mgr.is_on_last_section()

    def test_lap_count_increments(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        for i in range(2, 9):
            section = DEFAULT_SECTIONS[i - 1]
            wp = section.waypoints[-1]
            mgr.update(Pose(wp[0], wp[1]))
        assert mgr.get_lap_count() == 1

    def test_multiple_laps(self, mgr):
        for lap in range(3):
            mgr.update(Pose(0.1, 0.0))
            for i in range(2, 9):
                section = DEFAULT_SECTIONS[i - 1]
                wp = section.waypoints[-1]
                mgr.update(Pose(wp[0], wp[1]))
        assert mgr.get_lap_count() == 3

    def test_reset(self, mgr):
        mgr.update(Pose(0.1, 0.0))
        mgr.update(Pose(1.1, 0.0))
        mgr.reset()
        assert mgr.get_current_section() is None
        assert mgr.get_transition_count() == 0
        assert mgr.get_lap_count() == 0
        assert len(mgr.get_section_history()) == 0
