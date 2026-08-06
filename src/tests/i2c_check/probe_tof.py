# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: tests/i2c_check/probe_tof.py
# -----------------------------------------------------------------------------
# ToF measurement probe — run on the Pi AFTER a failing pre-flight:
#
#   python tests/i2c_check/probe_tof.py
#
# NON-INVASIVE: this probe NEVER re-programs I2C addresses, NEVER touches
# XSHUT, and NEVER re-inits the sensors. It only:
#   1. scans the bus,
#   2. dumps the result block (register 0x00) of every ToF-candidate
#      address that is present,
#   3. tries a few measurement-start sequences (register writes) and dumps
#      again — writes are limited to ToF-like addresses (0x29/0x2C/0x30/
#      0x31/0x32/0x60/0x62/0x64), never to 0x68 or unknown addresses.
#
# Symptom being diagnosed: "bad distance: 0.0". Addresses are already
# unique (e.g. 0x29, 0x30, 0x32) — the question is whether the result
# registers are populated at all, and in which byte layout:
#   VL53L0X style: range = bytes 10-11 of the block at 0x00
#   VL53L1X style: range = bytes 14-15, status = byte 16
#
# Everything printed is a raw fact from the bus — no simulation.
# =============================================================================

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from pi.system.logger import log

# Candidate ToF addresses (including clone mis-addressed variants 0x60/0x62/
# 0x64). 0x2C is dumped read-only (may be the magnetometer at a non-standard
# address — never write to it). 0x68 is the MPU6050, never touched.
TOF_CANDIDATES = (0x29, 0x2C, 0x30, 0x31, 0x32, 0x60, 0x62, 0x64)
WRITE_OK = (0x29, 0x30, 0x31, 0x32, 0x60, 0x62, 0x64)


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


def block(bus, addr):
    """Block read from reg 0x00. Returns (bytes, n) or (None, 0)."""
    for size in (17, 12):
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
        log.warn(f"  [{tag}] 0x{addr:02X}: block read FAILED (no ACK)")
        return
    row = " ".join(f"{b:02x}" for b in data[:16])
    extra = f"  +b16={data[16]:02x}" if n >= 17 else ""
    log.info(f"  [{tag}] 0x{addr:02X} ({n}B): {row}{extra}")
    b10b11 = (data[10] << 8) | data[11] if n >= 12 else None
    b14b15 = (data[14] << 8) | data[15] if n >= 15 else None
    if n >= 17:
        log.info(f"          L0X range(10-11)=0x{b10b11:04X} | "
                 f"L1X range(14-15)=0x{b14b15:04X} | status(16)=0x{data[16]:02X}")
    else:
        log.info(f"          L0X range(10-11)=0x{b10b11:04X}")


def identify(bus, addr):
    """Fingerprint registers: stop_variable + model id bytes."""
    vals = {r: rd(bus, addr, r) for r in (0x91, 0xC0, 0xC1, 0xC2)}
    shown = " ".join(f"0x{r:02X}={'--' if v is None else f'{v:02x}'}"
                     for r, v in vals.items())
    log.info(f"  identity 0x{addr:02X}: {shown}")


def trigger_l0x_single(bus, addr):
    """T3: VL53L0X single-shot start (reg 0x00 = 0x01)."""
    return wr(bus, addr, 0x00, 0x01)


def trigger_l0x_continuous(bus, addr):
    """T4: VL53L0X free-running continuous start (Pololu sequence)."""
    wr(bus, addr, 0x80, 0x01)
    wr(bus, addr, 0xFF, 0x01)
    wr(bus, addr, 0x00, 0x00)
    sv = rd(bus, addr, 0x91)
    log.info(f"    stop_variable 0x91 = 0x{sv:02x}" if sv is not None
             else "    stop_variable 0x91 read FAILED")
    if sv is None:
        sv = 0x00
    wr(bus, addr, 0x91, sv)
    wr(bus, addr, 0x00, 0x01)
    wr(bus, addr, 0xFF, 0x00)
    wr(bus, addr, 0x80, 0x00)
    return wr(bus, addr, 0x00, 0x02)


def trigger_l1x_mode_start(bus, addr):
    """T1: VL53L1X SYSTEM__MODE_START = 0x01 (reg 0x20)."""
    return wr(bus, addr, 0x20, 0x01)


def trigger_l1x_clear_and_start(bus, addr):
    """T2: clear interrupt (0x18) then MODE_START (0x20)."""
    wr(bus, addr, 0x18, 0x00)
    return wr(bus, addr, 0x20, 0x01)


def scan(bus):
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte_data(addr, 0)
            found.append(addr)
        except Exception:
            continue
    return found


def probe_address(bus, addr, writable):
    log.info(f"===== probe 0x{addr:02X} =====")
    identify(bus, addr)
    dump(bus, addr, "T0 no-trigger 1st")
    time.sleep(0.3)
    dump(bus, addr, "T0 no-trigger 2nd (auto-measure?)")
    if not writable:
        log.info("  (read-only probe — address not written to)")
        return
    attempts = [
        ("T3 L0X single-shot", trigger_l0x_single),
        ("T4 L0X continuous", trigger_l0x_continuous),
        ("T1 L1X mode-start", trigger_l1x_mode_start),
        ("T2 L1X clear+start", trigger_l1x_clear_and_start),
    ]
    for tag, fn in attempts:
        log.info(f"  -> trigger: {tag}")
        fn(bus, addr)
        time.sleep(0.35)
        data, n = block(bus, addr)
        if data is None:
            log.warn(f"  [{tag}] block read FAILED after trigger")
            continue
        b10b11 = (data[10] << 8) | data[11] if n >= 12 else None
        b14b15 = (data[14] << 8) | data[15] if n >= 15 else None
        nonzero = [i for i, b in enumerate(data) if b]
        if n >= 17:
            log.info(f"  [{tag}] 10-11=0x{b10b11:04X} 14-15=0x{b14b15:04X} "
                     f"16=0x{data[16]:02X}")
        else:
            log.info(f"  [{tag}] 10-11=0x{b10b11:04X}")
        log.info(f"          non-zero bytes: {nonzero}")
        time.sleep(0.1)


def main():
    log.init(name="WRO_TOF_PROBE")
    log.info("ToF measurement probe (non-invasive, no address changes)...")

    try:
        import smbus2
    except ImportError:
        log.error("smbus2 not available — this probe must run on the Pi")
        return

    bus = smbus2.SMBus(1)
    found = scan(bus)
    log.info(f"bus scan: {[f'0x{a:02X}' for a in found]}")

    present = [a for a in TOF_CANDIDATES if a in found]
    log.info(f"ToF candidates present: {[f'0x{a:02X}' for a in present]}")
    if not present:
        log.error("no ToF-candidate address responds — check wiring/power "
                  "(a dead bus shows only 0x68/0x0D, or nothing)")

    for addr in present:
        probe_address(bus, addr, addr in WRITE_OK)

    log.info("probe finished — send this output for analysis")
    bus.close()


if __name__ == "__main__":
    main()
