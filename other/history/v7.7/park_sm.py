import time
import math
import logging
from enum import Enum, auto


class ParkState(Enum):
    IDLE = auto()
    MARKER_SEEN = auto()
    BETWEEN_MARKERS = auto()
    ALIGNING = auto()
    BACKING_IN = auto()
    PARKED = auto()
    VERIFIED = auto()


class MarkerPosition:
    def __init__(self, x_px=0.0, y_px=0.0, marker_id=0):
        self.x_px = x_px
        self.y_px = y_px
        self.marker_id = marker_id

    def __repr__(self):
        return (f"Marker(id={self.marker_id}, "
                f"x={self.x_px:.0f}px, y={self.y_px:.0f}px)")


class ParkStateMachine:
    def __init__(self, alignment_tolerance=0.03, buffer_size=3,
                 centered_angle_deg=15, back_in_speed=0.08,
                 verify_time=1.0):
        self.alignment_tolerance = alignment_tolerance
        self.buffer_size = buffer_size
        self.centered_angle_deg = centered_angle_deg
        self.back_in_speed = back_in_speed
        self.verify_time = verify_time

        self.state = ParkState.IDLE
        self.state_start_time = 0.0
        self._left_buffer = []
        self._right_buffer = []
        self._avg_left = 0.0
        self._avg_right = 0.0
        self._markers = []
        self._last_aligned_check = False
        self._back_in_start_pose = None
        self._back_in_distance = 0.0

        self.logger = logging.getLogger(self.__class__.__name__)

    def _enter_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.monotonic()
        self.logger.info(f"Park state: {new_state.name}")

    def update(self, markers, tof_left, tof_right):
        self._markers = markers
        self._update_readings(tof_left, tof_right)

        if self.state == ParkState.IDLE:
            self._handle_idle()
        elif self.state == ParkState.MARKER_SEEN:
            self._handle_marker_seen()
        elif self.state == ParkState.BETWEEN_MARKERS:
            self._handle_between_markers()
        elif self.state == ParkState.ALIGNING:
            self._handle_aligning()
        elif self.state == ParkState.BACKING_IN:
            self._handle_backing_in()
        elif self.state == ParkState.PARKED:
            self._handle_parked()
        elif self.state == ParkState.VERIFIED:
            self._handle_verified()

    def _update_readings(self, tof_left, tof_right):
        self._left_buffer.append(tof_left)
        self._right_buffer.append(tof_right)
        if len(self._left_buffer) > self.buffer_size:
            self._left_buffer.pop(0)
        if len(self._right_buffer) > self.buffer_size:
            self._right_buffer.pop(0)
        if self._left_buffer:
            self._avg_left = sum(self._left_buffer) / len(self._left_buffer)
        if self._right_buffer:
            self._avg_right = sum(self._right_buffer) / len(self._right_buffer)

    def _handle_idle(self):
        if self._has_both_markers():
            self._enter_state(ParkState.MARKER_SEEN)

    def _handle_marker_seen(self):
        if self._check_centered():
            self._enter_state(ParkState.BETWEEN_MARKERS)

    def _handle_between_markers(self):
        elapsed = time.monotonic() - self.state_start_time
        if elapsed > 0.5:
            self._enter_state(ParkState.ALIGNING)

    def _handle_aligning(self):
        aligned = self._check_alignment()
        self._last_aligned_check = aligned
        if aligned:
            self._enter_state(ParkState.BACKING_IN)

    def _handle_backing_in(self):
        elapsed = time.monotonic() - self.state_start_time
        if elapsed > 2.0:
            self._enter_state(ParkState.PARKED)

    def _handle_parked(self):
        elapsed = time.monotonic() - self.state_start_time
        if elapsed > self.verify_time:
            self._enter_state(ParkState.VERIFIED)

    def _handle_verified(self):
        pass

    def _has_both_markers(self):
        return len(self._markers) >= 2

    def _check_centered(self):
        if len(self._markers) < 2:
            return False
        frame_center_x = 320
        threshold_px = frame_center_x * math.tan(
            math.radians(self.centered_angle_deg) / 45.0
        )
        for m in self._markers:
            offset = abs(m.x_px - frame_center_x)
            if offset > threshold_px:
                return False
        return True

    def _check_alignment(self):
        if not self._left_buffer or not self._right_buffer:
            return False
        diff = abs(self._avg_left - self._avg_right)
        aligned = diff < self.alignment_tolerance
        if not aligned:
            self.logger.debug(
                f"Alignment: L={self._avg_left:.3f}, "
                f"R={self._avg_right:.3f}, "
                f"diff={diff:.3f} > tol={self.alignment_tolerance}"
            )
        return aligned

    def trigger_parking(self):
        if self.state == ParkState.IDLE:
            self._enter_state(ParkState.MARKER_SEEN)

    def is_parking(self):
        return self.state != ParkState.IDLE

    def is_parked(self):
        return self.state in (ParkState.PARKED, ParkState.VERIFIED)

    def is_verified(self):
        return self.state == ParkState.VERIFIED

    def get_state(self):
        return self.state

    def get_state_time(self):
        return time.monotonic() - self.state_start_time

    def get_alignment_error(self):
        return abs(self._avg_left - self._avg_right)

    def get_marker_count(self):
        return len(self._markers)

    def reset(self):
        self.state = ParkState.IDLE
        self.state_start_time = 0.0
        self._left_buffer.clear()
        self._right_buffer.clear()
        self._avg_left = 0.0
        self._avg_right = 0.0
        self._markers.clear()
        self._last_aligned_check = False
        self._back_in_start_pose = None
        self._back_in_distance = 0.0
