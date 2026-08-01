# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/tof/family.py
# -----------------------------------------------------------------------------
# ToF silicon family detection and measurement start sequences.
#
# Why this exists: the modules labelled "VL53L0X" and "VL53L1X" on the robot
# do NOT always match their labels. The probe proved it empirically:
#   - the "VL53L1X" front module reads IDENTIFICATION registers 0xC0=0xEE,
#     0xC1=0xAA, 0xC2=0x10 — the exact VL53L0X fingerprint;
#   - the module at 0x29 streams live data with no trigger at all, which is
#     VL53L1X behaviour (a VL53L0X never measures until started).
#
# So each driver detects the real silicon before choosing:
#   - which measurement-start sequence to run, and
#   - which result-block layout to parse (VL53L0X: range at bytes 10-11 of
#     the block at 0x00; VL53L1X: range at bytes 14-15, status at byte 16).
#
# Detection is read-only and non-invasive: only registers are read, nothing
# is written, no address is changed.
# =============================================================================

import time


def detect_family(bus, address):
    """
    Identify the ToF silicon at `address` from its ID registers.

    Returns:
      "l0x"     - VL53L0X (0xC0 = 0xEE, 0xC1 = 0xAA, 0xC2 = 0x10).
      "l1x"     - VL53L1X (MODEL_ID at 0x010F = 0xEA, 0x010E = 0xCC).
      "unknown" - neither ID matched (clone with different ID bytes, or
                  not a ToF at all).
    """
    # VL53L0X: IDENTIFICATION__MODEL_ID = 0xEE (reg 0xC0).
    try:
        if bus.read_byte_data(address, 0xC0) == 0xEE:
            return "l0x"
    except Exception:
        pass
    # VL53L1X: MODEL_ID at 0x010F = 0xEA, 0x010E = 0xCC. The L1X uses 16-bit
    # register addresses (2 address bytes, MSB first), which smbus2's normal
    # API cannot send — use raw i2c_msg so the address is written correctly.
    try:
        from smbus2 import i2c_msg
        write = i2c_msg.write(address, [0x01, 0x0F])
        read = i2c_msg.read(address, 3)
        bus.i2c_rdwr(write, read)
        data = list(read)
        if 0xEA in data:
            return "l1x"
    except Exception:
        pass
    return "unknown"


def start_l0x_continuous(bus, address):
    """
    Start free-running continuous ranging on a VL53L0X (Pololu sequence).

    A VL53L0X does NOT measure by itself: the result registers stay 0 until
    the host starts the measurement. The known-good sequence has TWO prefix
    rounds — one before writing the mode, one AFTER. The first round takes
    the sensor out of reset and lets us read back the stop_variable (0x91);
    the second round is part of the official StartMeasurement and is easy
    to forget — without it the sensor stays idle (exactly the symptom the
    probe reproduced).

    Best-effort: writes are attempted and failures logged by the caller;
    nothing is raised.
    """
    def prefix():
        bus.write_byte_data(address, 0x80, 0x01)
        bus.write_byte_data(address, 0xFF, 0x01)
        bus.write_byte_data(address, 0x00, 0x00)
        try:
            sv = bus.read_byte_data(address, 0x91)
        except Exception:
            sv = 0x00
        bus.write_byte_data(address, 0x91, sv)
        bus.write_byte_data(address, 0x00, 0x01)
        bus.write_byte_data(address, 0xFF, 0x00)
        bus.write_byte_data(address, 0x80, 0x00)

    prefix()
    # Free-running continuous mode (no inter-measurement period).
    bus.write_byte_data(address, 0x00, 0x02)
    prefix()
    time.sleep(0.1)


def start_l1x(bus, address):
    """
    Best-effort start for a VL53L1X: SYSTEM__MODE_START (0x20) = 0x01.

    Evidence shows the L1X-family module ranges on its own after power-up,
    so this is a safety net rather than a requirement.
    """
    bus.write_byte_data(address, 0x20, 0x01)


def pick_range(bus, address, l0x_mm, family_kind):
    """
    Return the best range reading for the detected silicon family.

    Both layouts are always read (a 17-byte block at register 0x00):
      - VL53L0X layout: bytes 10-11.
      - VL53L1X layout: bytes 14-15.

    Preference follows the detected family:
      - "l0x"      : bytes 10-11 first (standard L0X result registers).
      - "l1x"/"unknown": bytes 14-15 first (the L1X-family module streams
        live data there; the probe showed its L0X-layout bytes are
        sporadic, so they are only a fallback).

    Returns a distance in mm, or None if both layouts read zero.
    """
    try:
        data17 = bus.read_i2c_block_data(address, 0x00, 17)
        l1x_mm = (data17[14] << 8) | data17[15]
    except Exception:
        return l0x_mm if l0x_mm > 0 else None
    if family_kind in ("l1x", "unknown"):
        if l1x_mm > 0:
            return l1x_mm
        return l0x_mm if l0x_mm > 0 else None
    # "l0x": standard layout first.
    if l0x_mm > 0:
        return l0x_mm
    return l1x_mm if l1x_mm > 0 else None
