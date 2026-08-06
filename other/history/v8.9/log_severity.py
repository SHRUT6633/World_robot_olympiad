from enum import IntEnum


class LogSeverity(IntEnum):
    DEBUG = 10
    INFO = 20
    WARNING = 30
    ERROR = 40
    CRITICAL = 50


SEVERITY_LABELS = {
    LogSeverity.DEBUG: "DEBUG",
    LogSeverity.INFO: "INFO",
    LogSeverity.WARNING: "WARN",
    LogSeverity.ERROR: "ERROR",
    LogSeverity.CRITICAL: "CRIT",
}


def format_severity(severity: LogSeverity) -> str:
    return SEVERITY_LABELS.get(severity, "UNKNOWN")


SEVERITY_RATE_LIMITS_S = {
    LogSeverity.DEBUG: 2.0,
    LogSeverity.INFO: 2.0,
    LogSeverity.WARNING: 2.0,
    LogSeverity.ERROR: 5.0,
    LogSeverity.CRITICAL: 0.0,
}


def get_rate_limit(severity: LogSeverity) -> float:
    return SEVERITY_RATE_LIMITS_S.get(severity, 2.0)
