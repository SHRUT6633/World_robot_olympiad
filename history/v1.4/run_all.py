#!/usr/bin/env python3
"""
WRO Future Engineers 4WS - Single-Command Launcher
Modes:  boot  - Full boot: self-test + LED + switch -> race
        selftest - Run all sensor/module tests, report results
        race  - Skip self-test, directly start race logic
        pi    - Run Pi control loop only
        esp   - Flash ESP32-S3 firmware
        sim   - Simulation mode
        full  - Pi + ESP32 combined
        test  - Run pytest suite
"""

import sys
import os
import subprocess
import argparse
from pathlib import Path


def print_header():
    print("=" * 60)
    print("  WRO Future Engineers - Autonomous 4WS Robot")
    print("  Raspberry Pi 4B + ESP32-S3 + Single Servo 4WS")
    print("=" * 60)


def check_dependencies():
    missing = []
    try:
        import numpy
    except ImportError:
        missing.append("numpy")
    try:
        import cv2
    except ImportError:
        missing.append("opencv-python")
    try:
        import serial
    except ImportError:
        missing.append("pyserial")
    if missing:
        print(f"[!] Missing: {', '.join(missing)}")
        print("[*] Run: pip install -r requirements.txt")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="WRO 4WS Robot Launcher")
    parser.add_argument("--mode",
        choices=["boot", "selftest", "race", "pi", "esp", "sim", "full", "test"],
        default="boot",
        help="boot (default, full + LED + switch), selftest, race, pi, esp, sim, full, test")
    parser.add_argument("--config", default="config/pi_config.yaml")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARN", "ERROR"])
    args = parser.parse_args()

    print_header()
    os.environ["WRO_CONFIG"] = args.config
    os.environ["WRO_UART_PORT"] = args.port
    os.environ["WRO_LOG_LEVEL"] = args.log_level
    os.chdir(Path(__file__).parent)

    # ===== TEST MODE =====
    if args.mode == "test":
        print("[*] Running pytest...")
        sys.exit(subprocess.call([sys.executable, "-m", "pytest", "tests/", "-v"]))

    # ===== ESP FLASH MODE =====
    if args.mode == "esp":
        print("[*] ESP32 flash mode")
        subprocess.call(["idf.py", "-C", "esp", "flash"], shell=(os.name == "nt"))
        return

    # ===== SELF-TEST MODE (no LED/Switch, just CLI report) =====
    if args.mode == "selftest":
        check_dependencies()
        from pi.system.logger import Logger
        Logger().init(name="WRO_SELFTEST", level=args.log_level)

        from pi.boot import power_on_self_test
        results, runner = power_on_self_test()
        if results["failed"] == 0:
            print("\n[OK] ALL SELF-TESTS PASSED")
            sys.exit(0)
        else:
            print(f"\n[FAIL] {results['failed']} test(s) failed")
            sys.exit(1)

    # ===== BOOT MODE (self-test + LED + switch -> race) =====
    if args.mode == "boot":
        check_dependencies()
        from pi.system.logger import Logger
        Logger().init(name="WRO_BOOT", level=args.log_level, log_dir="logs")

        from pi.boot import boot_sequence
        success = boot_sequence()
        if not success:
            print("\n[FAIL] Boot aborted - check logs")
            sys.exit(1)
        return

    # ===== SIMULATION MODE =====
    if args.mode == "sim":
        print("[*] Simulation mode")
        from pi.main import main as pi_main
        import asyncio
        asyncio.run(pi_main())
        return

    # ===== RACE / PI / FULL MODES =====
    if args.mode in ("race", "pi", "full"):
        if not check_dependencies():
            return
        from pi.system.logger import log as logger
        logger.init(level=args.log_level)

        if args.mode == "full":
            from pi.comm.uart import UARTCommunicator
            logger.info("Full mode: Pi + ESP32 UART enabled")

        from pi.main import main as pi_main
        import asyncio
        try:
            asyncio.run(pi_main())
        except KeyboardInterrupt:
            print("\n[*] Shutdown")
        except Exception as e:
            print(f"[!] Fatal: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)


if __name__ == "__main__":
    main()
