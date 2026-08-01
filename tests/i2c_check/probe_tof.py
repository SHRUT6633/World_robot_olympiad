# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: tests/i2c_check/probe_tof.py
# -----------------------------------------------------------------------------
# ToF measurement probe — run on the Pi AFTER a failing pre-flight:
#
#   python tests/i2c_check/probe_tof.py
#
# Symptom being diagnosed: "bad distance: 0.0" from the clone ToF modules.
# The drivers program the I2C address correctly (0x30 / 0x32 respond) but the
# result registers read all-zero. Two things are unknown about the clones:
#
#   1. Do they need an explicit "start measurement" command before the result
#      block (register 0x00) is populated?
#   2. Which result layout do they use — VL53L0X style (range at bytes 10-11)
#      or VL53L1X style (range at bytes 14-15)?
#
# This probe answers both empirically: it initialises the sensors with the
# exact same drivers as the checker, then dumps the result block after each
# candidate trigger sequence. It prints the raw bytes so the layout and the
# working trigger are obvious from the data.
#
# It never fakes anything: every register read/write is attempted on the real
# bus and reported as it happened.
# =============================================================================

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from pi.system.config_manager import ConfigManager
from pi.system.logger import log
from pi.sensors.tof.vl53l0x import VL53L0X
from pi.sensors.tof.vl53l1x import VL53L1X


def wr(bus, addr, reg, val):
    """Write one byte; report success/failure truthfully."""
    try:
        bus.write_byte_data(addr, reg, val)
        return True
    except Exception as e:
        log.warn(f"    write 0x{addr:02X} reg 0x{reg:02X}=0x{val:02X} FAILED - {e}")
        return False


def rd(bus, addr, reg):
    """Read one byte; return None on error."""
    try:
        return bus.read_byte_data(addr, reg)
    except Exception:
        return None


def block(bus, addr, n=17):
    """Block read of n bytes from reg 0x00. Returns (bytes, got_n)."""
    for size in (17, 12):
        if n > size:
            continue
        try:
            data = bus.read_i2c_block_data(addr, 0x00, size)
            return data, size
        except Exception:
            continue
    return None, 0


def dump(bus, addr, tag):
    """Annotated dump of the result block at register 0x00."""
    data, n = block(bus, addr)
    if data is None:
        log.warn(f"  [{tag}] block read at 0x{addr:02X} FAILED (no ACK)")
        return
    row = " ".join(f"{b:02x}" for b in data[:16])
    extra = f"  +byte16={data[16]:02x}" if n >= 17 else ""
    log.info(f"  [{tag}] 0x{addr:02X} block({n}B): {row}{extra}")
    b10b11 = (data[10] << 8) | data[11] if n >= 12 else None
    b14b15 = (data[14] << 8) | data[15] if n >= 15 else None
    if n >= 17:
        log.info(f"          L0X layout range(10-11)=0x{b10b11:04X} | "
                 f"L1X layout range(14-15)=0x{b14b15:04X} "
                 f"| status(16)=0x{data[16]:02X}")
    else:
        log.info(f"          L0X layout range(10-11)=0x{b10b11:04X}")


def identify(bus, addr):
    """Fingerprint registers: model id + stop_variable."""
    vals = {}
    for reg in (0x91, 0xC0, 0xC1, 0xC2):
        vals[reg] = rd(bus, addr, reg)
    log.info(f"  identity 0x{addr:02X}: 0x91={vals[0x91]} 0xC0={vals[0xC0]} "
             f"0xC1={vals[0xC1]} 0xC2={vals[0xC2]}")


def trigger_l0x_single(bus, addr):
    """T3: VL53L0X single-shot start (reg 0x00 = 0x01)."""
    return wr(bus, addr, 0x00, 0x01)


def trigger_l0x_continuous(bus, addr):
    """T4: VL53L0X free-running continuous start (Pololu sequence)."""
    wr(bus, addr, 0x80, 0x01)
    wr(bus, addr, 0xFF, 0x01)
    wr(bus, addr, 0x00, 0x00)
    sv = rd(bus, addr, 0x91)
    log.info(f"    stop_variable 0x91 read = 0x{sv:02X}" if sv is not None
             else "    stop_variable 0x91 read FAILED")
    if sv is None:
        sv = 0x00
    wr(bus, addr, 0x91, sv)
    wr(bus, addr, 0x00, 0x01)
    wr(bus, addr, 0xFF, 0x00)
    wr(bus, addr, 0x80, 0x00)
    return wr(bus, addr, 0x00, 0x02)


def trigger_l1x_mode_start(bus, addr):
    """T1: VL53L1X SYSTEM__MODE_START = 0x01 (reg 0x0020)."""
    return wr(bus, addr, 0x20, 0x01)


def trigger_l1x_clear_and_start(bus, addr):
    """T2: clear interrupt (0x0018) then MODE_START (0x0020)."""
    wr(bus, addr, 0x18, 0x00)
    return wr(bus, addr, 0x20, 0x01)


def probe_sensor(bus, sensor, label):
    addr = sensor.address
    log.info(f"--- probing {label} @ 0x{addr:02X} ---")
    identify(bus, addr)
    dump(bus, addr, "T0 no-trigger 1st")
    time.sleep(0.3)
    dump(bus, addr, "T0 no-trigger 2nd (auto-measure?)")

    attempts = [
        ("T3 L0X single-shot", trigger_l0x_single),
        ("T4 L0X continuous", trigger_l0x_continuous),
        ("T1 L1X mode-start", trigger_l1x_mode_start),
        ("T2 L1X clear+start", trigger_l1x_clear_and_start),
    ]
    for tag, fn in attempts:
        log.info(f"  -> trigger: {tag}")
        ok = fn(bus, addr)
        time.sleep(0.35)
        data, n = block(bus, addr)
        if data is None:
            log.warn(f"  [{tag}] block read FAILED after trigger")
            continue
        b10b11 = (data[10] << 8) | data[11] if n >= 12 else None
        b14b15 = (data[14] << 8) | data[15] if n >= 15 else None
        nonzero = [i for i, b in enumerate(data) if b]
        if n >= 17:
            log.info(f"  [{tag}] after trigger: 10-11=0x{b10b11:04X} "
                     f"14-15=0x{b14b15:04X} 16=0x{data[16]:02X}")
        else:
            log.info(f"  [{tag}] after trigger: 10-11=0x{b10b11:04X}")
        log.info(f"          non-zero bytes: {nonzero}")
        time.sleep(0.1)


def main():
    log.init(name="WRO_TOF_PROBE")
    log.info("ToF measurement probe starting...")

    config = ConfigManager()
    config.load()

    try:
        import smbus2
    except ImportError:
        log.error("smbus2 not available — this probe must run on the Pi")
        return

    c = config
    sensors = {
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
    }

    ready = {}
    for name, sensor in sensors.items():
        try:
            sensor.init()
            ready[name] = sensor
            log.info(f"{name}: init OK @ 0x{sensor.address:02X}")
        except Exception as e:
            log.error(f"{name}: init FAILED - {e} (hardware wiring issue, "
                      f"probe continues with the rest)")

    if not ready:
        log.error("no ToF sensor initialised — nothing to probe; "
                  "check wiring/XSHUT first")
        return

    bus = smbus2.SMBus(1)

    if "tof_left" in ready:
        probe_sensor(bus, ready["tof_left"], "tof_left (VL53L0X driver)")
    if "tof_front" in ready:
        probe_sensor(bus, ready["tof_front"], "tof_front (VL53L1X driver)")
    if "tof_right" not in ready:
        log.warn("tof_right never initialised — check XSHUT wire on GPIO 27")

    log.info("probe finished — send this output for analysis")
    bus.close()
    for sensor in ready.values():
        try:
            sensor.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
