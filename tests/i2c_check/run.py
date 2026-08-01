# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: tests/i2c_check/run.py
# -----------------------------------------------------------------------------
# I2C pre-flight gate: entry point.
#
#   python tests/i2c_check/run.py
#
# Flow:
#   1. Runs the I2C pre-flight check on every sensor (ToF x3, MPU6050,
#      QMC5883L) using the same config as pi/main.py.
#   2. ALL OK   -> green LED blinks 3x and stays green, then the FULL
#                  SYSTEM starts automatically (python pi/main.py).
#   3. ANY FAIL -> red LED on, failures listed, system NOT started.
# =============================================================================

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from pi.system.config_manager import ConfigManager
from pi.system.logger import log
from pi.hardware.led import StatusLED
from checker import I2CChecker


def main():
    log.init(name="WRO_I2C_CHECK")
    log.info("I2C pre-flight check starting...")

    config = ConfigManager()
    config.load()

    led = StatusLED(
        green_pin=config.get("hardware", "leds", "green_pin", default=23),
        red_pin=config.get("hardware", "leds", "red_pin", default=24),
    )

    checker = I2CChecker(config)
    results = checker.run_all()

    failed = [name for name, (ok, _) in results.items() if not ok]

    if not failed:
        # ------------------------------------------------------------------
        # ALL TESTS PASSED -> green LED, then start the whole system
        # ------------------------------------------------------------------
        log.info("=" * 50)
        log.info("  ALL I2C TESTS PASSED - GREEN LED BLINK")
        log.info("  Starting full system...")
        log.info("=" * 50)
        led.success_sequence()   # 3x green blink, then solid green
        led.close()
        checker.close()
        try:
            subprocess.run([sys.executable, "pi/main.py"], cwd=str(ROOT))
        except KeyboardInterrupt:
            log.info("System stopped by user")
        except Exception as e:
            log.critical(f"Failed to start system: {e}")
            sys.exit(1)
    else:
        # ------------------------------------------------------------------
        # TESTS FAILED -> red LED, do NOT start the system
        # ------------------------------------------------------------------
        log.error("=" * 50)
        log.error(f"  I2C CHECK FAILED - {len(failed)} FAULTY: {failed}")
        for name in failed:
            log.error(f"    {name}: {results[name][1]}")
        log.error("  Fix hardware, then re-run. RED LED ON.")
        log.error("=" * 50)
        led.set("red")
        checker.close()
        sys.exit(1)


if __name__ == "__main__":
    main()
