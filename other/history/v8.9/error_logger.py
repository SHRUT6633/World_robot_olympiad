import time
import logging
from enum import IntEnum
from dataclasses import dataclass, field

logger = logging.getLogger("error_logger")


class Severity(IntEnum):
    DEBUG = 10
    INFO = 20
    WARNING = 30
    ERROR = 40
    CRITICAL = 50


@dataclass
class SourceState:
    name: str
    last_log_time: float = 0.0
    consecutive_failures: int = 0
    suppressed_count: int = 0
    disabled: bool = False


@dataclass
class ErrorLoggerConfig:
    rate_limit_interval_s: float = 2.0
    error_rate_limit_s: float = 5.0
    max_consecutive_failures: int = 50


class ErrorLogger:
    def __init__(self, config: ErrorLoggerConfig | None = None):
        self.config = config or ErrorLoggerConfig()
        self._sources: dict[str, SourceState] = {}

    def log(
        self,
        source: str,
        message: str,
        severity: Severity = Severity.ERROR,
    ) -> bool:
        if source not in self._sources:
            self._sources[source] = SourceState(name=source)

        state = self._sources[source]

        if state.disabled:
            return False

        if severity == Severity.CRITICAL:
            self._write(source, message, severity)
            return True

        now = time.monotonic()
        rate = self.config.error_rate_limit_s if severity >= Severity.ERROR else self.config.rate_limit_interval_s

        if now - state.last_log_time < rate:
            state.suppressed_count += 1
            state.consecutive_failures += 1

            if state.consecutive_failures >= self.config.max_consecutive_failures:
                state.disabled = True
                logger.warning(
                    f"Source '{source}' auto-disabled after "
                    f"{state.consecutive_failures} consecutive failures"
                )
            return False

        if state.suppressed_count > 0:
            logger.info(
                f"Suppressed {state.suppressed_count} messages "
                f"from '{source}' in last {rate:.1f}s"
            )

        self._write(source, message, severity)
        state.last_log_time = now
        state.suppressed_count = 0
        state.consecutive_failures = 0
        return True

    def _write(self, source: str, message: str, severity: Severity):
        level = {
            Severity.DEBUG: logging.DEBUG,
            Severity.INFO: logging.INFO,
            Severity.WARNING: logging.WARNING,
            Severity.ERROR: logging.ERROR,
            Severity.CRITICAL: logging.CRITICAL,
        }.get(severity, logging.INFO)
        logger.log(level, f"[{source}] {message}")

    def reset_source(self, source: str):
        state = self._sources.get(source)
        if state:
            state.disabled = False
            state.consecutive_failures = 0
            state.suppressed_count = 0

    def get_disabled_sources(self) -> list[str]:
        return [n for n, s in self._sources.items() if s.disabled]

    def get_stats(self) -> dict:
        return {
            name: {
                "disabled": s.disabled,
                "suppressed": s.suppressed_count,
                "consecutive_failures": s.consecutive_failures,
            }
            for name, s in self._sources.items()
        }
