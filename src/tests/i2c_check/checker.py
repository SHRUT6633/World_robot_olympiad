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
# CHANGES vs original:
#   - Up-front config validation: warns loudly, before touching any
#     hardware, if multiple ToF sensors share a bus with no xshut_pin
#     configured. This is the #1 cause of address collisions and it used
#     to only show up as confusing downstream errors ("silicon ID
#     unknown", "no chip found").
#   - Stale-address detection: if a target address (0x30/0x31/0x32) is
#     already ACKing on the BEFORE-init scan, that means the sensor
#     state survived from a previous run and the board needs a real
#     power cycle (VL53L0X/L1X address assignment is volatile RAM).
#   - scan_bus now uses a plain read_byte() presence check instead of
#     read_byte_data(addr, 0) — some real chips will NACK a register-0
#     read even while physically present and healthy, causing false
#     "not found" results in the scan table.
#   - Data reads get a couple of quick retries before being declared
#     FAILED — ToF sensors especially can return one noisy/invalid
#     sample right after init before settling.
#   - Final summary prints an expected-address-vs-actually-found table,
#     so a missing/misplaced sensor is obvious without reading raw hex
#     grids.
#
# Usage (from run.py):
#   checker = I2CChecker(config)
#   results = checker.run_all()   # {name: (ok: bool, detail: str)}
#   checker.close()
# =============================================================================

import time
import numpy as np
from pi.system.config_manager import ConfigManager
from pi.system.logger import log
from pi.sensors.tof.vl53l0x import VL53L0X
from pi.sensors.tof.vl53l1x import VL53L1X
from pi.sensors.imu.mpu6050 import MPU6050
from pi.sensors.magnetometer.qmc5883l import QMC5883L


class I2CChecker:
    # Config path for each ToF sensor: (name -> config key tuple)
    TOF_CONFIG_KEYS = {
        "tof_left":  ("sensors", "vl53l0x_left"),
        "tof_right": ("sensors", "vl53l0x_right"),
        "tof_front": ("sensors", "vl53l1x_front"),
    }

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
    # NEW: catch the #1 cause of ToF collisions before touching hardware
    # ---------------------------------------------------------------------
    def check_xshut_config(self):
        by_bus = {}
        for name, keys in self.TOF_CONFIG_KEYS.items():
            bus = self.config.get(*keys, "i2c_bus", default=1)
            xshut = self.config.get(*keys, "xshut_pin", default=None)
            by_bus.setdefault(bus, []).append((name, xshut))

        problem_found = False
        for bus, sensors_on_bus in by_bus.items():
            missing = [n for n, x in sensors_on_bus if x is None]
            if len(sensors_on_bus) > 1 and missing:
                problem_found = True
                names = ", ".join(missing)
                log.error(
                    f"CONFIG PROBLEM: {len(sensors_on_bus)} ToF sensors share "
                    f"i2c-{bus} but these have no xshut_pin set: {names}. "
                    f"Every VL53L0X/VL53L1X boots at the SAME default address "
                    f"(0x29) — without a dedicated, controllable XSHUT GPIO "
                    f"per sensor they will collide on the bus. Set "
                    f"sensors.<name>.xshut_pin in your config for each one "
                    f"sharing this bus before trusting any result below."
                )
        return not problem_found

    # ---------------------------------------------------------------------
    # NEW: warn if a target address is already alive before init — means
    # the sensor never actually reset (no power cycle since last run)
    # ---------------------------------------------------------------------
    def check_stale_addresses(self, before_found_bus1):
        c = self.config
        target_addrs = {
            "tof_left":  c.get("sensors", "vl53l0x_left", "address", default=0x30),
            "tof_right": c.get("sensors", "vl53l0x_right", "address", default=0x31),
            "tof_front": c.get("sensors", "vl53l1x_front", "address", default=0x32),
        }
        for name, addr in target_addrs.items():
            if addr in before_found_bus1:
                log.warn(
                    f"STALE STATE: target address {hex(addr)} for {name} is "
                    f"already responding BEFORE init. VL53L0X/L1X address "
                    f"assignment lives in volatile RAM and only clears on a "
                    f"real power cycle (or XSHUT low->high pulse) — this "
                    f"sensor's state carried over from a previous run/crash. "
                    f"If results below look wrong, power-cycle the sensors "
                    f"and re-run."
                )

    # ---------------------------------------------------------------------
    # Raw bus scan (like i2cdetect -y 1) — reports which addresses ACK
    # ---------------------------------------------------------------------
    def scan_bus(self, bus_num=1):
        try:
            import smbus2
        except ImportError:
            log.warn("scan_bus: smbus2 not available")
            return []
        try:
            dev = smbus2.SMBus(bus_num)
        except Exception as e:
            log.warn(f"scan_bus: cannot open /dev/i2c-{bus_num}: {e}")
            return []
        found = []
        for addr in range(0x03, 0x78):
            try:
                # Plain presence check — more reliable than read_byte_data(addr, 0),
                # which some real chips NACK even when physically present.
                dev.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
            except Exception:
                pass
        dev.close()
        return found

    def scan_all_buses(self):
        """Scan bus 1 (GPIO2/3 header pins 3/5) and bus 0 (pins 27/28)."""
        return {
            "i2c-1 (SDA=GPIO2, SCL=GPIO3)": self.scan_bus(1),
            "i2c-0 (SDA=GPIO0, SCL=GPIO1)": self.scan_bus(0),
        }

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
    # NEW: retry a sensor's read() a couple times before giving up —
    # covers the "first sample right after init is noisy" case.
    # ---------------------------------------------------------------------
    def _read_with_retry(self, sensor, attempts=3, delay=0.08):
        last_exc = None
        for _ in range(attempts):
            try:
                return sensor.read(), None
            except Exception as e:
                last_exc = e
                time.sleep(delay)
        return None, last_exc

    # ---------------------------------------------------------------------
    # Run every check
    # ---------------------------------------------------------------------
    def run_all(self):
        log.info("=" * 50)
        log.info("  I2C PRE-FLIGHT CHECK")
        log.info("=" * 50)

        # 0. Config sanity check — catches the most common root cause
        #    before we waste time on confusing hardware-level errors.
        self.check_xshut_config()

        # 1. Scan BEFORE init (all ToF still at factory 0x29, ideally)
        before_scans = self.scan_all_buses()
        for bus_label, found in before_scans.items():
            log.info(f"Bus scan before init ({bus_label}):\n"
                     f"{self.format_scan(found)}")
        self.check_stale_addresses(before_scans.get("i2c-1 (SDA=GPIO2, SCL=GPIO3)", []))

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

        # 3. Data check on every sensor that initialised (with retry)
        for name, sensor in self.sensors.items():
            if not self.results.get(name, (False, ""))[0]:
                continue
            data, exc = self._read_with_retry(sensor)
            if exc is not None:
                log.error(f"  {name}: read FAILED - {exc}")
                self.results[name] = (False, str(exc))
                continue
            ok, detail = self._validate(name, data)
            if ok:
                log.info(f"  {name}: data OK ({detail})")
                self.results[name] = (True, detail)
            else:
                log.error(f"  {name}: data check FAILED ({detail})")
                self.results[name] = (False, detail)

        # 4. Scan AFTER init (expect 0x30 0x31 0x32 0x68 0x0D on bus 1)
        after_scans = self.scan_all_buses()
        for bus_label, found in after_scans.items():
            log.info(f"Bus scan after init ({bus_label}):\n"
                     f"{self.format_scan(found)}")

        # 5. NEW: expected-vs-actual address summary
        self._log_address_summary(after_scans)

        return self.results

    # ---------------------------------------------------------------------
    # NEW: one-glance table of "who's supposed to be where" vs reality
    # ---------------------------------------------------------------------
    def _log_address_summary(self, after_scans):
        c = self.config
        expected = {
            "tof_left":  c.get("sensors", "vl53l0x_left", "address", default=0x30),
            "tof_right": c.get("sensors", "vl53l0x_right", "address", default=0x31),
            "tof_front": c.get("sensors", "vl53l1x_front", "address", default=0x32),
            "imu":       c.get("sensors", "mpu6050", "address", default=0x68),
            "mag":       c.get("sensors", "qmc5883l", "address", default=0x0D),
        }
        all_found = set()
        for found in after_scans.values():
            all_found.update(found)

        log.info("-" * 50)
        log.info("  EXPECTED vs ACTUAL ADDRESS MAP")
        log.info("-" * 50)
        for name, addr in expected.items():
            present = addr in all_found
            marker = "OK on bus" if present else "MISSING from bus"
            log.info(f"    {name:<10} expected {hex(addr):<6} -> {marker}")
        log.info("-" * 50)

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