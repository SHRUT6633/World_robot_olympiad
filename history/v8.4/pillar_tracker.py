import time
from enum import Enum
from dataclasses import dataclass, field


class PassSide(Enum):
    LEFT = "left"
    RIGHT = "right"


@dataclass
class PillarObservation:
    pillar_id: str
    color: str
    bearing_deg: float
    distance_m: float
    timestamp: float


@dataclass
class PillarPassEvent:
    pillar_id: str
    color: str
    side: PassSide
    timestamp: float


@dataclass
class PillarTrackerConfig:
    cooldown_s: float = 0.5
    bearing_cross_threshold_deg: float = 5.0


class PillarTracker:
    def __init__(self, config: PillarTrackerConfig | None = None):
        self.config = config or PillarTrackerConfig()
        self._last_pass_time: dict[str, float] = {}
        self._previous_bearing: dict[str, float] = {}
        self._events: list[PillarPassEvent] = []
        self._pillar_count: int = 0

    def update(self, observation: PillarObservation) -> PillarPassEvent | None:
        key = observation.pillar_id
        now = observation.timestamp

        last_time = self._last_pass_time.get(key, 0.0)
        if now - last_time < self.config.cooldown_s:
            return None

        prev = self._previous_bearing.get(key, None)
        current = observation.bearing_deg
        self._previous_bearing[key] = current

        if prev is None:
            return None

        if prev > self.config.bearing_cross_threshold_deg and current <= -self.config.bearing_cross_threshold_deg:
            side = PassSide.RIGHT
        elif prev < -self.config.bearing_cross_threshold_deg and current >= self.config.bearing_cross_threshold_deg:
            side = PassSide.LEFT
        else:
            return None

        self._last_pass_time[key] = now

        event = PillarPassEvent(
            pillar_id=observation.pillar_id,
            color=observation.color,
            side=side,
            timestamp=now,
        )
        self._events.append(event)
        self._pillar_count += 1
        return event

    def validate_pass_side(self, event: PillarPassEvent, expected_side: PassSide) -> bool:
        return event.side == expected_side

    def get_total_pillars_passed(self) -> int:
        return self._pillar_count

    def reset(self):
        self._last_pass_time.clear()
        self._previous_bearing.clear()
        self._events.clear()
        self._pillar_count = 0
