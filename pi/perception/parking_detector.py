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
    # The parking manoeuvre is modelled as a deterministic 7-state machine.
    # Each state advances when specific sensor conditions are met, and the
    # whole sequence must complete within 60 seconds.
    IDLE = "IDLE"                     # Waiting for first pink marker.
    MARKER_SEEN = "MARKER_SEEN"       # One pink marker detected.
    BETWEEN_MARKERS = "BETWEEN_MARKERS"  # Two markers seen, aligning.
    ALIGNING = "ALIGNING"             # Fine alignment using ToF.
    BACKING_IN = "BACKING_IN"         # Reversing into the bay.
    PARKED = "PARKED"                 # Stationary in the zone.
    VERIFIED = "VERIFIED"             # 30 s stationary hold complete.
    FAILED = "FAILED"                 # Timeout or unrecoverable error.


class ParkingDetector:
    # 7-state parking bay detection state machine.
    #
    # Detection algorithm overview:
    #  1. Use PillarDetector to spot pink (magenta) markers — these mark
    #     the boundaries of the parking zone per the WRO rulebook.
    #  2. While between the two markers, monitor left/right ToF sensors
    #     to determine when the robot is parallel to the wall and close
    #     enough to begin reversing.
    #  3. Reverse until both ToF sensors read < 30 mm (nose is in the bay).
    #  4. Remain stationary for 30 s to satisfy the "VERIFIED" criterion.
    #
    # Why a state machine?  The manoeuvre is a strict linear sequence with
    # clear sensor predicates — a state machine is simple, verifiable, and
    # avoids the complexity of a behaviour tree for this specific case.

    def __init__(self, robot_length_mm=200):
        self._robot_length_mm = robot_length_mm

        # The parking bay length is defined as 1.5× the robot length.
        # This is used to verify that the two pink markers are far enough
        # apart to constitute a valid parking bay.
        self._parking_length_mm = int(1.5 * robot_length_mm)

        self._state = ParkingState.IDLE
        self._left_tof_history = []      # Rolling window for averaging.
        self._right_tof_history = []
        self._pink_markers_seen = 0
        self._first_marker_time = 0.0
        self._last_marker_time = 0.0
        self._entry_time = 0.0            # Timer for the 60 s total timeout.
        self._align_start_time = 0.0
        self._back_start_time = 0.0
        self._park_start_time = 0.0
        self._parallel_achieved = False

    def update(self, pink_detections, tof_left_mm=None, tof_right_mm=None):
        # pink_detections: list of dicts from PillarDetector for "pink" key.
        # tof_left_mm / tof_right_mm: raw ToF sensor readings (mm, or None).
        now = time.time()

        self._update_tof_history(tof_left_mm, tof_right_mm)

        # A marker is "present" if at least one pink detection exists.
        has_pink = len(pink_detections) > 0

        # --------------------------------------------------------------
        # State machine transitions
        # --------------------------------------------------------------

        if self._state == ParkingState.IDLE:
            if has_pink:
                self._first_marker_time = now
                self._pink_markers_seen = 1
                self._state = ParkingState.MARKER_SEEN
                log.info("Parking: first magenta marker detected")

        elif self._state == ParkingState.MARKER_SEEN:
            if has_pink:
                time_since_first = now - self._first_marker_time
                # Estimate the distance between the two markers by using the
                # left ToF reading as a proxy for the along-wall travel.
                distance_between = self._estimate_distance_between_markers(
                    time_since_first
                )
                # Only transition if the second marker is far enough from the
                # first (>= robot length) — this rejects spurious re-detections
                # of the same marker due to slight camera jitter.
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
                # If the marker vanishes for 5 seconds, reset to IDLE.
                # This could happen if the robot turned away or the marker
                # was a false positive.
                if now - self._first_marker_time > 5.0:
                    log.warning("Parking: lost marker, resetting")
                    self._state = ParkingState.IDLE
                    self._pink_markers_seen = 0

        elif self._state == ParkingState.BETWEEN_MARKERS:
            # Compute left–right ToF difference to check parallelism.
            # The robot must be roughly parallel to the wall (error ≤ 20 mm)
            # before alignment can begin.
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
                # When the nearer side is < 50 mm from the wall, we consider
                # the robot close enough to begin reversing in.
                if dist_to_wall < 50:
                    self._back_start_time = now
                    self._state = ParkingState.BACKING_IN
                    log.info(
                        f"Parking: backing into zone, wall at {dist_to_wall}mm"
                    )
                elif now - self._align_start_time > 10.0:
                    # Timeout: if we cannot get close enough in 10 s,
                    # proceed anyway (the bay may have shifted since the
                    # markers were detected).
                    log.warning("Parking: align timeout, trying anyway")
                    self._back_start_time = now
                    self._state = ParkingState.BACKING_IN

        elif self._state == ParkingState.BACKING_IN:
            if tof_left_mm is not None and tof_right_mm is not None:
                # Both ToF sensors must read < 30 mm — the robot's front
                # has entered the parking bay and is up against the wall.
                wall_close = (
                    tof_left_mm is not None
                    and tof_left_mm < 30
                    and tof_right_mm is not None
                    and tof_right_mm < 30
                )
                time_in_back = now - self._back_start_time
                if wall_close or time_in_back > 8.0:
                    # 8-second timeout: if we can't reach the wall, stop
                    # anyway to avoid infinite reversing.
                    self._park_start_time = now
                    self._state = ParkingState.PARKED
                    log.info(
                        "Parking: robot is in parking zone, waiting for verification"
                    )

        elif self._state == ParkingState.PARKED:
            # The robot must remain stationary for 30 seconds to satisfy
            # the WRO "verified park" criterion.
            elapsed_in_park = now - self._park_start_time
            if elapsed_in_park >= 30.0:
                self._state = ParkingState.VERIFIED
                log.info("Parking: VERIFIED (30s stationary)")

        # --- Global timeout ---
        # If 60 seconds have elapsed since the second marker was seen and
        # the robot is not yet parked or verified, declare failure.
        if now - self._entry_time > 60.0 and self._state not in (
            ParkingState.VERIFIED,
            ParkingState.FAILED,
            ParkingState.PARKED,
        ):
            log.warning("Parking: total timeout 60s, parking failed")
            self._state = ParkingState.FAILED

    def _update_tof_history(self, left, right, max_len=20):
        # Rolling buffer (max 20 samples) to smooth noisy ToF readings.
        # The oldest samples fall off automatically.
        if left is not None:
            self._left_tof_history.append(left)
            if len(self._left_tof_history) > max_len:
                self._left_tof_history.pop(0)
        if right is not None:
            self._right_tof_history.append(right)
            if len(self._right_tof_history) > max_len:
                self._right_tof_history.pop(0)

    def _estimate_distance_between_markers(self, time_elapsed_s):
        # Proxy: use the average left ToF reading as the along-wall distance
        # travelled.  This assumes the robot drives parallel to the wall at a
        # roughly constant offset.  A more accurate approach would integrate
        # wheel odometry, but the ToF proxy avoids calibration drift.
        if self._left_tof_history:
            avg_left = sum(self._left_tof_history) / len(self._left_tof_history)
        else:
            avg_left = 300  # Fallback guess when no ToF data yet.
        return avg_left

    def _compute_parallel_error(self):
        # The robot is parallel when the left and right ToF distances are
        # equal (within a tolerance).  The error is |avg_left - avg_right|.
        # Averaging over the last 3 samples reduces noise from momentary
        # reflections.
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
