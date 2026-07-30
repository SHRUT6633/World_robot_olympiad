# =============================================================================
# logger.py — Logger (Singleton Logging Utility)
# =============================================================================
# Provides a singleton Logger class that wraps Python's logging module with:
#   - Console output (stdout) with formatted timestamps and levels.
#   - Rotating file handler (max 5 MB per file, 3 backups) to disk.
#   - Convenience methods: debug(), info(), warn(), error(), critical().
#
# The module also creates a global instance `log` at import time:
#   from pi.system.logger import log
#   log.info("message")
#
# This `log` object is the common logger used throughout the entire
# codebase — in manager.py, boot.py, scheduler.py, health_monitor.py,
# performance_monitor.py, diagnostics.py, and every other module.
#
# Singleton behavior:
#   Logger() always returns the same instance. The .init() method is
#   idempotent: it only configures the underlying logging.Logger once.
#   Subsequent calls to .init() are ignored (see _initialized flag).
#
# Configuration parameters (passed to .init()):
#   name    – Logger name (also used as the log filename).
#             Default: "WRO_4WS". boot.py uses "WRO_BOOT".
#   level   – Logging level string: "DEBUG", "INFO", "WARNING", "ERROR",
#             "CRITICAL". Default: "INFO". Change to "DEBUG" for verbose
#             output during development.
#   log_dir – Directory where log files are written. Default: "logs".
#             The directory is created if it does not exist.
#
# File rotation:
#   maxBytes=5*1024*1024 (~5 MB) per file. backupCount=3 keeps 3 rotated
#   files (total ~20 MB max). If the robot runs for hours and logs heavily,
#   old logs are automatically deleted.
#
# Impact of changing parameters:
#   name     → changes the .log filename and the logger name in log records.
#   level    → setting "ERROR" suppresses info/warn/debug (quiet boot);
#              setting "DEBUG" shows every heartbeat (noisy but useful).
#   log_dir  → if "logs" is not writable, file handler creation fails
#              silently (Python logging swallows handler errors by default).
# =============================================================================

import logging
import sys
from pathlib import Path
from logging.handlers import RotatingFileHandler


class Logger:
    # Singleton instance storage.
    _instance = None

    # -------------------------------------------------------------------------
    # __new__(cls)
    # -------------------------------------------------------------------------
    # Singleton pattern: only one Logger ever exists.
    # The _initialized flag prevents .init() from re-configuring the
    # underlying Python logger on subsequent calls.
    # -------------------------------------------------------------------------
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    # -------------------------------------------------------------------------
    # init(name, level, log_dir)
    # -------------------------------------------------------------------------
    # Configures the Python logging.Logger once. If _initialized is True,
    # this does nothing (idempotent).
    #
    #   1. Creates a logger with the given name (getLogger is singleton
    #      within Python's logging module, so re-calling with the same
    #      name returns the same logger).
    #   2. Sets the logging level (default: INFO).
    #   3. Creates a formatter:  "HH:MM:SS.mmm | LEVEL     | name | message"
    #   4. Adds a StreamHandler that writes to stdout.
    #   5. Creates the log_dir if it doesn't exist.
    #   6. Adds a RotatingFileHandler(maxBytes=5MB, backupCount=3).
    #
    # If this method is never called, the Logger instance has no underlying
    # Python logger and calls to .info() etc. will silently do nothing.
    #
    # Called by:
    #   - boot.py:   logger.init(name="WRO_BOOT", level="INFO", log_dir="logs")
    #   - Any other module that wants to re-initialize (harmless if redundant).
    # -------------------------------------------------------------------------
    def init(self, name: str = "WRO_4WS", level: str = "INFO", log_dir: str = "logs"):
        if self._initialized:
            return
        self._initialized = True

        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper(), logging.INFO))

        # Format: "HH:MM:SS.mmm | INFO     | WRO_4WS | Your message here"
        fmt = logging.Formatter(
            "%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)s | %(message)s",
            datefmt="%H:%M:%S",
        )

        # Console handler (stdout)
        sh = logging.StreamHandler(sys.stdout)
        sh.setFormatter(fmt)
        self.logger.addHandler(sh)

        # File handler with rotation
        Path(log_dir).mkdir(exist_ok=True)
        fh = RotatingFileHandler(
            f"{log_dir}/{name}.log", maxBytes=5 * 1024 * 1024, backupCount=3
        )
        fh.setFormatter(fmt)
        self.logger.addHandler(fh)

    # -------------------------------------------------------------------------
    # Convenience methods
    # -------------------------------------------------------------------------
    # Each delegates to the corresponding logging.Logger method.
    # If .init() was never called, self.logger doesn't exist → AttributeError.
    # In practice, .init() is always called by boot.py or the first user.
    # -------------------------------------------------------------------------
    def debug(self, msg, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        # Note: logging uses "warning" (lowercase), but we expose "warn"
        # for brevity (matching Python's deprecated warnings.warn style).
        self.logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs)


# =============================================================================
# Global Singleton Instance
# =============================================================================
# Every module in the project uses this shared instance:
#   from pi.system.logger import log
#   log.info(...)
#
# Note: .init() must be called before any log output appears. boot.py calls
# it early. If a module logs before boot.py calls .init(), those messages
# are silently dropped.
# =============================================================================
log = Logger()
