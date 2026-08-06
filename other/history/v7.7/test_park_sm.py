import pytest
import math
import time
from park_sm import ParkStateMachine, ParkState, MarkerPosition


class TestParkStateMachine:
    @pytest.fixture
    def sm(self):
        return ParkStateMachine(
            alignment_tolerance=0.03,
            buffer_size=3,
            centered_angle_deg=15,
            verify_time=0.1,
        )

    def test_initial_idle(self, sm):
        assert sm.get_state() == ParkState.IDLE
        assert not sm.is_parking()
        assert not sm.is_parked()

    def test_markers_seen_transitions(self, sm):
        markers = [
            MarkerPosition(100, 240, 1),
            MarkerPosition(540, 240, 2),
        ]
        sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.MARKER_SEEN
        assert sm.is_parking()

    def test_no_markers_stays_idle(self, sm):
        sm.update([], 0.30, 0.30)
        assert sm.get_state() == ParkState.IDLE

    def test_one_marker_stays_idle(self, sm):
        markers = [MarkerPosition(320, 240, 1)]
        sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.IDLE

    def test_centered_check_passes(self, sm):
        markers = [
            MarkerPosition(280, 240, 1),
            MarkerPosition(360, 240, 2),
        ]
        sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.MARKER_SEEN

        markers2 = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        sm.update(markers2, 0.30, 0.30)
        assert sm.get_state() == ParkState.BETWEEN_MARKERS

    def test_offcenter_markers_dont_transition(self, sm):
        markers = [
            MarkerPosition(100, 240, 1),
            MarkerPosition(540, 240, 2),
        ]
        sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.MARKER_SEEN

        sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.MARKER_SEEN

    def test_alignment_uses_averaged_readings(self, sm):
        markers = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.28)

        for _ in range(5):
            sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.ALIGNING

        for _ in range(3):
            sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.BACKING_IN

    def test_aligning_to_backing_in(self, sm):
        markers = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.BACKING_IN

    def test_parked_after_backing(self, sm):
        markers = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.30)
        sm.update(markers, 0.30, 0.30)

        assert sm.get_state() == ParkState.BACKING_IN

        for _ in range(10):
            sm.update(markers, 0.30, 0.30)
        assert sm.get_state() == ParkState.PARKED

    def test_verified_after_parked(self, sm):
        markers = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        for _ in range(20):
            sm.update(markers, 0.30, 0.30)
        assert sm.is_verified()

    def test_alignment_fails_no_buffers(self, sm):
        assert sm.get_alignment_error() == 0.0

    def test_sensor_noise_averaging(self, sm):
        markers = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        import random
        random.seed(42)
        for _ in range(3):
            noise_l = 0.30 + random.uniform(-0.03, 0.03)
            noise_r = 0.30 + random.uniform(-0.03, 0.03)
            sm.update(markers, noise_l, noise_r)

        aligned = sm._check_alignment()
        assert abs(sm._avg_left - sm._avg_right) < abs(noise_l - noise_r)

    def test_trigger_parking(self, sm):
        sm.trigger_parking()
        assert sm.get_state() == ParkState.MARKER_SEEN

    def test_reset(self, sm):
        markers = [
            MarkerPosition(290, 240, 1),
            MarkerPosition(350, 240, 2),
        ]
        for _ in range(20):
            sm.update(markers, 0.30, 0.30)
        assert sm.is_verified()
        sm.reset()
        assert sm.get_state() == ParkState.IDLE
        assert not sm.is_parking()
        assert len(sm._left_buffer) == 0
