# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/localization/track_map.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Track section mapping for lap counting and corner detection
# =============================================================================

from ..system.logger import log


class TrackSection:
    STRAIGHT = "STRAIGHT"
    CORNER = "CORNER"
    OBSTACLE_ZONE = "OBSTACLE_ZONE"
    NARROW = "NARROW"
    PARKING_ZONE = "PARKING_ZONE"
    START_FINISH = "START_FINISH"
    UNKNOWN = "UNKNOWN"


class CornerDirection:
    LEFT = "LEFT"
    RIGHT = "RIGHT"


class TrackMap:
    def __init__(self, track_width_mm=1000, outer_size_mm=3000, inner_size_mm=1000):
        self._track_width_mm = track_width_mm
        self._outer = outer_size_mm
        self._inner = inner_size_mm
        self._centerline_len = (self._outer + self._inner) / 2
        self._lap_distance_mm = 4 * self._centerline_len
        self._current_section = TrackSection.START_FINISH
        self._section_start_distance = 0.0
        self._total_distance_mm = 0.0
        self._current_corner = 1
        self._corner_direction = CornerDirection.LEFT
        self._corner_start_dist = 0.0
        self._in_corner = False
        self._obstacles_passed = 0
        self._lap_count = 0
        self._last_lap_distance = 0.0
        self._section_map = self._build_section_map()

    def _build_section_map(self):
        cl = self._centerline_len
        return [
            (TrackSection.START_FINISH, 0, 0.15 * cl),
            (TrackSection.STRAIGHT, 0.15 * cl, 0.85 * cl),
            (TrackSection.CORNER, 0.85 * cl, 1.15 * cl),
            (TrackSection.STRAIGHT, 1.15 * cl, 1.85 * cl),
            (TrackSection.CORNER, 1.85 * cl, 2.15 * cl),
            (TrackSection.STRAIGHT, 2.15 * cl, 2.85 * cl),
            (TrackSection.CORNER, 2.85 * cl, 3.15 * cl),
            (TrackSection.STRAIGHT, 3.15 * cl, 3.85 * cl),
            (TrackSection.CORNER, 3.85 * cl, 4.0 * cl),
        ]

    def update(self, distance_travelled_mm, yaw=None):
        self._total_distance_mm += distance_travelled_mm

        if self._total_distance_mm >= self._lap_distance_mm:
            self._total_distance_mm -= self._lap_distance_mm
            self._lap_count += 1
            log.info(f"TrackMap: lap {self._lap_count} completed")

        dist_in_lap = self._total_distance_mm
        for section, start, end in self._section_map:
            if start <= dist_in_lap < end:
                if section != self._current_section:
                    old = self._current_section
                    self._current_section = section
                    self._section_start_distance = dist_in_lap
                    log.info(
                        f"TrackMap: {old} -> {section} "
                        f"(lap dist {dist_in_lap:.0f}mm)"
                    )
                break

    def is_in_section(self, section):
        return self._current_section == section

    def travelling_straight(self):
        return self._current_section == TrackSection.STRAIGHT

    def in_corner(self):
        return self._current_section == TrackSection.CORNER

    def at_parking_zone(self):
        return self._current_section == TrackSection.PARKING_ZONE

    def current_section(self):
        return self._current_section

    def progress_in_section(self):
        dist_in_lap = self._total_distance_mm
        for section, start, end in self._section_map:
            if start <= dist_in_lap < end:
                if end > start:
                    return (dist_in_lap - start) / (end - start)
                return 0.0
        return 0.0

    def lap_count(self):
        return self._lap_count

    def corner_number(self):
        dist_in_lap = self._total_distance_mm
        cl = self._centerline_len
        corner_starts = [0.85 * cl, 1.85 * cl, 2.85 * cl, 3.85 * cl]
        for i, cs in enumerate(corner_starts):
            if dist_in_lap >= cs and dist_in_lap < cs + 0.3 * cl:
                return i + 1
        return None

    def set_obstacle_zone(self):
        self._current_section = TrackSection.OBSTACLE_ZONE

    def pass_obstacle(self):
        self._obstacles_passed += 1
        log.info(f"TrackMap: obstacle {self._obstacles_passed} passed")

    def obstacles_passed(self):
        return self._obstacles_passed

    def total_distance_mm(self):
        return self._total_distance_mm

    def reset(self):
        self._total_distance_mm = 0.0
        self._current_section = TrackSection.START_FINISH
        self._lap_count = 0
        self._obstacles_passed = 0
