# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/parking_detector.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Parking bay detection and state machine
# =============================================================================

import time
from ..system.logger import log


class ParkingState:
    IDLE = "IDLE"
    MARKER_SEEN = "MARKER_SEEN"
    BETWEEN_MARKERS = "BETWEEN_MARKERS"
    ALIGNING = "ALIGNING"
    BACKING_IN = "BACKING_IN"
    PARKED = "PARKED"
    VERIFIED = "VERIFIED"
    FAILED = "FAILED"


class ParkingDetector:
    def __init__(self, robot_length_mm=200):
        self._robot_length_mm = robot_length_mm
        self._parking_length_mm = int(1.5 * robot_length_mm)
        self._state = ParkingState.IDLE
        self._left_tof_history = []
        self._right_tof_history = []
        self._pink_markers_seen = 0
        self._first_marker_time = 0.0
        self._last_marker_time = 0.0
        self._entry_time = 0.0
        self._align_start_time = 0.0
        self._back_start_time = 0.0
        self._park_start_time = 0.0
        self._parallel_achieved = False

    def update(self, pink_detections, tof_left_mm=None, tof_right_mm=None):
        now = time.time()
        self._update_tof_history(tof_left_mm, tof_right_mm)
        has_pink = len(pink_detections) > 0

        if self._state == ParkingState.IDLE:
            if has_pink:
                self._first_marker_time = now
                self._pink_markers_seen = 1
                self._state = ParkingState.MARKER_SEEN
                log.info("Parking: first magenta marker detected")

        elif self._state == ParkingState.MARKER_SEEN:
            if has_pink:
                time_since_first = now - self._first_marker_time
                distance_between = self._estimate_distance_between_markers(
                    time_since_first
                )
                if (
                    distance_between
                    and distance_between >= self._robot_length_mm
                ):
                    self._pink_markers_seen = 2
                    self._last_marker_time = now
                    self._entry_time = now
                    self._state = ParkingState.BETWEEN_MARKERS
                    log.info(
                        f"Parking: second marker, between markers, "
                        f"parking length {self._parking_length_mm}mm"
                    )
            else:
                if now - self._first_marker_time > 5.0:
                    log.warning("Parking: lost marker, resetting")
                    self._state = ParkingState.IDLE
                    self._pink_markers_seen = 0

        elif self._state == ParkingState.BETWEEN_MARKERS:
            parallel_error = self._compute_parallel_error()
            if parallel_error is not None and parallel_error <= 20:
                self._parallel_achieved = True
                self._align_start_time = now
                self._state = ParkingState.ALIGNING
                log.info(
                    f"Parking: parallel alignment OK ({parallel_error:.0f}mm)"
                )

        elif self._state == ParkingState.ALIGNING:
            if tof_left_mm is not None and tof_right_mm is not None:
                dist_to_wall = min(tof_left_mm, tof_right_mm)
                if dist_to_wall < 50:
                    self._back_start_time = now
                    self._state = ParkingState.BACKING_IN
                    log.info(
                        f"Parking: backing into zone, wall at {dist_to_wall}mm"
                    )
                elif now - self._align_start_time > 10.0:
                    log.warning("Parking: align timeout, trying anyway")
                    self._back_start_time = now
                    self._state = ParkingState.BACKING_IN

        elif self._state == ParkingState.BACKING_IN:
            if tof_left_mm is not None and tof_right_mm is not None:
                wall_close = (
                    tof_left_mm is not None
                    and tof_left_mm < 30
                    and tof_right_mm is not None
                    and tof_right_mm < 30
                )
                time_in_back = now - self._back_start_time
                if wall_close or time_in_back > 8.0:
                    self._park_start_time = now
                    self._state = ParkingState.PARKED
                    log.info(
                        "Parking: robot is in parking zone, waiting for verification"
                    )

        elif self._state == ParkingState.PARKED:
            elapsed_in_park = now - self._park_start_time
            if elapsed_in_park >= 30.0:
                self._state = ParkingState.VERIFIED
                log.info("Parking: VERIFIED (30s stationary)")

        if now - self._entry_time > 60.0 and self._state not in (
            ParkingState.VERIFIED,
            ParkingState.FAILED,
            ParkingState.PARKED,
        ):
            log.warning("Parking: total timeout 60s, parking failed")
            self._state = ParkingState.FAILED

    def _update_tof_history(self, left, right, max_len=20):
        if left is not None:
            self._left_tof_history.append(left)
            if len(self._left_tof_history) > max_len:
                self._left_tof_history.pop(0)
        if right is not None:
            self._right_tof_history.append(right)
            if len(self._right_tof_history) > max_len:
                self._right_tof_history.pop(0)

    def _estimate_distance_between_markers(self, time_elapsed_s):
        if self._left_tof_history:
            avg_left = sum(self._left_tof_history) / len(self._left_tof_history)
        else:
            avg_left = 300
        return avg_left

    def _compute_parallel_error(self):
        if (
            len(self._left_tof_history) < 3
            or len(self._right_tof_history) < 3
        ):
            return None
        avg_left = sum(self._left_tof_history[-3:]) / 3
        avg_right = sum(self._right_tof_history[-3:]) / 3
        return abs(avg_left - avg_right)

    def state(self):
        return self._state

    def is_parked(self):
        return self._state in (
            ParkingState.PARKED,
            ParkingState.VERIFIED,
        )

    def is_verified(self):
        return self._state == ParkingState.VERIFIED

    def parallel_error_mm(self):
        return self._compute_parallel_error()

    def reset(self):
        self._state = ParkingState.IDLE
        self._left_tof_history.clear()
        self._right_tof_history.clear()
        self._pink_markers_seen = 0
        self._parallel_achieved = False
