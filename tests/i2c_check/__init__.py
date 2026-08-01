# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: tests/i2c_check/__init__.py
# -----------------------------------------------------------------------------
# I2C pre-flight check package.
#
#   python tests/i2c_check/run.py
#
# Verifies all I2C sensors (ToF left/right/front, MPU6050, QMC5883L).
# All OK  -> green LED blink, then full system starts.
# Any fail -> red LED on, system stays stopped.
# =============================================================================

from .checker import I2CChecker

__all__ = ["I2CChecker"]
