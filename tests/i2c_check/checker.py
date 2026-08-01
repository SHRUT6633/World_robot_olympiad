# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: tests/i2c_check/checker.py
# -----------------------------------------------------------------------------
# I2C pre-flight checker: scans the I2C bus and verifies every sensor
# (ToF left/right/front, MPU6050, QMC5883L) using the exact same config
# and drivers as pi/main.py.
#
# A check PASSES only if:
#   1. The sensor initialises on REAL hardware (mock mode = FAIL, because
#      mock means no hardware was found).
#   2. A real data read succeeds (distance in range / accel+gyro finite /
#      magnetometer not stuck at zero).
#
# Usage (from run.py): 
#   checker = I2CChecker(config)
#   results = checker.run_all()   # {name: (ok: bool, detail: str)}
#   checker.close()
# =============================================================================

import numpy as np
from pi.system.config_manager import ConfigManager
from pi.system.logger import log
from pi.sensors.tof.vl53l0x import VL53L0X
from pi.sensors.tof.vl53l1x import VL53L1X
from pi.sensors.imu.mpu6050 import MPU6050
from pi.sensors.magnetometer.qmc5883l import QMC5883L


class I2CChecker:
    def __init__(self, config=None):
        self.config = config if config is not None else ConfigManager()
        self.results = {}
        self.sensors = {}

    # ---------------------------------------------------------------------
    # Build the same sensor objects (same config keys) as pi/main.py
    # ---------------------------------------------------------------------
    def build_sensors(self):
        c = self.config
        self.sensors = {
            "tof_left": VL53L0X(
                "VL53L0X_Left",
                bus=c.get("sensors", "vl53l0x_left", "i2c_bus", default=1),
                address=c.get("sensors", "vl53l0x_left", "address", default=0x30),
                xshut_pin=c.get("sensors", "vl53l0x_left", "xshut_pin", default=None),
            ),
            "tof_right": VL53L0X(
                "VL53L0X_Right",
                bus=c.get("sensors", "vl53l0x_right", "i2c_bus", default=1),
                address=c.get("sensors", "vl53l0x_right", "address", default=0x31),
                xshut_pin=c.get("sensors", "vl53l0x_right", "xshut_pin", default=None),
            ),
            "tof_front": VL53L1X(
                "VL53L1X_Front",
                bus=c.get("sensors", "vl53l1x_front", "i2c_bus", default=1),
                address=c.get("sensors", "vl53l1x_front", "address", default=0x32),
                xshut_pin=c.get("sensors", "vl53l1x_front", "xshut_pin", default=None),
            ),
            "imu": MPU6050(
                bus=c.get("sensors", "mpu6050", "i2c_bus", default=1),
                address=c.get("sensors", "mpu6050", "address", default=0x68),
                accel_range=c.get("sensors", "mpu6050", "accel_range", default=4),
                gyro_range=c.get("sensors", "mpu6050", "gyro_range", default=500),
            ),
            "mag": QMC5883L(
                bus=c.get("sensors", "qmc5883l", "i2c_bus", default=1),
                address=c.get("sensors", "qmc5883l", "address", default=0x0D),
            ),
        }
        return self.sensors

    # ---------------------------------------------------------------------
    # Raw bus scan (like i2cdetect -y 1) — reports which addresses ACK
    # ---------------------------------------------------------------------
    def scan_bus(self):
        try:
            import smbus2
        except ImportError:
            log.warn("scan_bus: smbus2 not available")
            return []
        bus_num = self.config.get("sensors", "vl53l0x_left", "i2c_bus", default=1)
        try:
            dev = smbus2.SMBus(bus_num)
        except Exception as e:
            log.warn(f"scan_bus: cannot open /dev/i2c-{bus_num}: {e}")
            return []
        found = []
        for addr in range(0x03, 0x78):
            try:
                dev.read_byte_data(addr, 0)
                found.append(addr)
            except OSError:
                pass
            except Exception:
                pass
        dev.close()
        return found

    def format_scan(self, found):
        lines = ["      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f"]
        for row in range(0x00, 0x08):
            line = f"  {row:02x}: "
            for col in range(16):
                addr = row * 16 + col
                line += f"{addr:02x} " if addr in found else "-- "
            lines.append(line)
        return "\n".join(lines)

    # ---------------------------------------------------------------------
    # Run every check
    # ---------------------------------------------------------------------
    def run_all(self):
        log.info("=" * 50)
        log.info("  I2C PRE-FLIGHT CHECK")
        log.info("=" * 50)

        # 1. Scan BEFORE init (all ToF still at factory 0x29)
        try:
            before = self.scan_bus()
            log.info(f"Bus scan before init:\n{self.format_scan(before)}")
        except Exception as e:
            before = []
            log.error(f"Bus scan failed: {e}")

        # 2. Build + init every sensor (drivers now verify real hardware;
        #    they raise on missing chips, so FAILED is always truthful)
        self.build_sensors()
        for name, sensor in self.sensors.items():
            try:
                sensor.init()
                if getattr(sensor, "_device", None) == "mock":
                    log.error(f"  {name}: FAILED - mock mode (no real hardware)")
                    self.results[name] = (False, "mock mode (no real hardware)")
                else:
                    log.info(f"  {name}: init OK")
                    self.results[name] = (True, "init OK")
            except Exception as e:
                log.error(f"  {name}: FAILED - {e}")
                self.results[name] = (False, str(e))

        # 3. Data check on every sensor that initialised
        for name, sensor in self.sensors.items():
            if not self.results.get(name, (False, ""))[0]:
                continue
            try:
                data = sensor.read()
                ok, detail = self._validate(name, data)
                if ok:
                    log.info(f"  {name}: data OK ({detail})")
                    self.results[name] = (True, detail)
                else:
                    log.error(f"  {name}: data check FAILED ({detail})")
                    self.results[name] = (False, detail)
            except Exception as e:
                log.error(f"  {name}: read FAILED - {e}")
                self.results[name] = (False, str(e))

        # 4. Scan AFTER init (expect 0x30 0x31 0x32 0x68 0x0D)
        try:
            after = self.scan_bus()
            log.info(f"Bus scan after init:\n{self.format_scan(after)}")
        except Exception as e:
            log.error(f"Bus scan failed: {e}")

        return self.results

    # ---------------------------------------------------------------------
    # Per-sensor data validation
    # ---------------------------------------------------------------------
    def _validate(self, name, data):
        if name.startswith("tof"):
            # distance must be a real, in-range millimetre value
            if data is None or not (0 < data < 5000):
                return False, f"bad distance: {data}"
            return True, f"{data:.0f} mm"

        if name == "imu":
            if not isinstance(data, dict) or "accel" not in data or "gyro" not in data:
                return False, "no accel/gyro data"
            if not (np.all(np.isfinite(data["accel"]))
                    and np.all(np.isfinite(data["gyro"]))):
                return False, "non-finite values"
            return True, (f"accelZ={data['accel'][2]:.2f}g "
                          f"gyroZ={data['gyro'][2]:.1f}deg/s")

        if name == "mag":
            if data is None or not np.all(np.isfinite(data)):
                return False, "no data"
            # a powered magnetometer always reads Earth's field (hundreds
            # of counts) — all-axes-near-zero means a dead/stuck chip
            if np.all(np.abs(data) < 1):
                return False, f"stuck near zero: {data}"
            return True, f"({data[0]:.0f}, {data[1]:.0f}, {data[2]:.0f})"

        return False, "unknown sensor"

    def close(self):
        for sensor in self.sensors.values():
            try:
                sensor.close()
            except Exception:
                pass
