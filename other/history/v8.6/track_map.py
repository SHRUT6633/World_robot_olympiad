import math
from dataclasses import dataclass, field


@dataclass
class TrackSection:
    name: str
    start_distance_m: float
    end_distance_m: float
    expected_behavior: str


TRACK_SECTIONS = [
    TrackSection("start_straight", 0.0, 1.5, "accelerate_to_cruise"),
    TrackSection("first_curve", 1.5, 3.0, "steer_same_phase_r0.8"),
    TrackSection("mid_straight", 3.0, 4.5, "maintain_speed"),
    TrackSection("pillar_zone", 4.5, 6.0, "detect_and_avoid_pillars"),
    TrackSection("second_curve", 6.0, 7.5, "steer_same_phase_r0.8"),
    TrackSection("parking_approach", 7.5, 8.5, "reduce_speed_switch_to_opposite"),
    TrackSection("parking_zone", 8.5, 9.2, "detect_markers_parallel_park"),
]


@dataclass
class TrackPosition:
    lap: int = 0
    distance_m: float = 0.0
    section: str = "start_straight"
    section_progress: float = 0.0
    is_start_finish: bool = False


class TrackMap:
    def __init__(self, track_length_m: float = 9.2):
        self._track_length = track_length_m
        self._sections = TRACK_SECTIONS
        self._distance_m = 0.0
        self._current_lap = 0
        self._calibration_factor = 1.0
        self._lap_start_errors: list[float] = []

    def update(self, delta_distance_m: float, start_finish: bool) -> TrackPosition:
        if start_finish and self._distance_m > 0.5:
            error = self._distance_m - self._track_length
            self._lap_start_errors.append(error)
            self._distance_m = 0.0
            self._current_lap += 1

            if len(self._lap_start_errors) >= 3:
                avg_error = sum(self._lap_start_errors) / len(self._lap_start_errors)
                correction = -avg_error / self._track_length * 0.001
                self._calibration_factor += correction
                self._calibration_factor = max(0.95, min(1.05, self._calibration_factor))
                self._lap_start_errors = []

        self._distance_m += delta_distance_m * self._calibration_factor

        if self._distance_m > self._track_length:
            self._distance_m -= self._track_length
            self._current_lap += 1

        current_section = self._sections[-1]
        for sec in self._sections:
            if sec.start_distance_m <= self._distance_m < sec.end_distance_m:
                current_section = sec
                break

        progress = 0.0
        if current_section.end_distance_m > current_section.start_distance_m:
            progress = (
                (self._distance_m - current_section.start_distance_m)
                / (current_section.end_distance_m - current_section.start_distance_m)
            )

        return TrackPosition(
            lap=self._current_lap,
            distance_m=self._distance_m,
            section=current_section.name,
            section_progress=min(1.0, max(0.0, progress)),
            is_start_finish=start_finish,
        )

    def reset(self):
        self._distance_m = 0.0
        self._current_lap = 0
