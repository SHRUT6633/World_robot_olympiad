import time
import logging
from enum import Enum
from dataclasses import dataclass, field

logger = logging.getLogger("health_monitor")


class TaskHealth(Enum):
    ALIVE = "alive"
    WARNING = "warning"
    DEAD = "dead"


@dataclass
class HeartbeatEntry:
    name: str
    last_heartbeat: float = 0.0
    missed_beats: int = 0
    health: TaskHealth = TaskHealth.ALIVE


@dataclass
class HealthMonitorConfig:
    allowed_missed_beats: int = 3
    check_interval_s: float = 0.1


class HealthMonitor:
    def __init__(self, config: HealthMonitorConfig | None = None):
        self.config = config or HealthMonitorConfig()
        self._entries: dict[str, HeartbeatEntry] = {}
        self._periods: dict[str, float] = {}
        self._emergency_stop = False

    def register_task(self, name: str, rate_hz: float):
        self._entries[name] = HeartbeatEntry(name=name)
        self._periods[name] = 1.0 / rate_hz

    def heartbeat(self, name: str):
        entry = self._entries.get(name)
        if entry is None:
            return
        entry.last_heartbeat = time.monotonic()
        entry.missed_beats = 0
        entry.health = TaskHealth.ALIVE

    def check(self):
        now = time.monotonic()
        for name, entry in self._entries.items():
            period = self._periods.get(name, 1.0)
            timeout = period * self.config.allowed_missed_beats
            elapsed = now - entry.last_heartbeat

            if elapsed > timeout:
                entry.missed_beats += 1
                if entry.missed_beats >= self.config.allowed_missed_beats:
                    entry.health = TaskHealth.DEAD
                    logger.error(
                        f"Task '{name}' declared DEAD — "
                        f"missed {entry.missed_beats} heartbeats"
                    )
                    self._emergency_stop = True
                else:
                    entry.health = TaskHealth.WARNING
                    logger.warning(
                        f"Task '{name}' heartbeat overdue: {elapsed*1000:.0f}ms"
                    )

    def is_emergency_stop(self) -> bool:
        return self._emergency_stop

    def get_health(self, name: str) -> TaskHealth | None:
        entry = self._entries.get(name)
        return entry.health if entry else None

    def get_all_health(self) -> dict[str, TaskHealth]:
        return {n: e.health for n, e in self._entries.items()}

    def reset(self):
        self._entries.clear()
        self._periods.clear()
        self._emergency_stop = False
