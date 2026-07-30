import time
import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Marker2D:
    x_m: float
    y_m: float
    id: int


@dataclass
class ParkingZone:
    center_x_m: float
    center_y_m: float
    orientation_deg: float
    width_m: float
    length_m: float
    left_distance_m: float
    right_distance_m: float
    markers_found: int


@dataclass
class ParkingDetectorConfig:
    parallel_threshold_mm: float = 20.0
    exposure_adapt_delay_s: float = 2.0
    baseline_exposure_comp: float = 0.0
    max_exposure_comp: float = 3.0


class ParkingDetector:
    def __init__(self, config: ParkingDetectorConfig | None = None):
        self.config = config or ParkingDetectorConfig()
        self._last_detection_time = time.monotonic()
        self._exposure_comp = config.baseline_exposure_comp if config else 0.0

    def detect_markers(self, frame: Any) -> list[Marker2D]:
        now = time.monotonic()
        markers = self._extract_markers(frame)

        if len(markers) == 0:
            elapsed = now - self._last_detection_time
            if elapsed > self.config.exposure_adapt_delay_s:
                self._exposure_comp = min(
                    self._exposure_comp + 1.0,
                    self.config.max_exposure_comp,
                )
        else:
            self._last_detection_time = now
            self._exposure_comp = self.config.baseline_exposure_comp

        return markers

    def compute_zone(self, markers: list[Marker2D]) -> ParkingZone | None:
        if len(markers) < 2:
            return None

        markers_sorted = sorted(markers, key=lambda m: m.id)

        if len(markers_sorted) >= 4:
            return self._compute_from_four(markers_sorted[:4])
        else:
            return self._estimate_from_partial(markers_sorted)

    def verify_parallel(self, zone: ParkingZone) -> bool:
        diff_mm = abs(zone.left_distance_m - zone.right_distance_m) * 1000.0
        return diff_mm <= self.config.parallel_threshold_mm

    def _extract_markers(self, frame: Any) -> list[Marker2D]:
        return []

    def _compute_from_four(self, markers: list[Marker2D]) -> ParkingZone:
        xs = [m.x_m for m in markers]
        ys = [m.y_m for m in markers]
        cx = sum(xs) / 4.0
        cy = sum(ys) / 4.0
        left_dist = abs(markers[0].y_m - markers[1].y_m) / 2.0
        right_dist = abs(markers[2].y_m - markers[3].y_m) / 2.0
        ang = math.degrees(math.atan2(ys[1] - ys[0], xs[1] - xs[0]))
        return ParkingZone(
            center_x_m=cx, center_y_m=cy, orientation_deg=ang,
            width_m=0.5, length_m=0.3,
            left_distance_m=left_dist, right_distance_m=right_dist,
            markers_found=4,
        )

    def _estimate_from_partial(self, markers: list[Marker2D]) -> ParkingZone:
        xs = [m.x_m for m in markers]
        ys = [m.y_m for m in markers]
        cx = sum(xs) / len(markers)
        cy = sum(ys) / len(markers)
        return ParkingZone(
            center_x_m=cx, center_y_m=cy, orientation_deg=0.0,
            width_m=0.5, length_m=0.3,
            left_distance_m=0.15, right_distance_m=0.15,
            markers_found=len(markers),
        )

    def get_exposure_compensation(self) -> float:
        return self._exposure_comp
