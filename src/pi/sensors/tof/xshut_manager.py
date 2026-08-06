# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/tof/xshut_manager.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Shared XSHUT sequencing for multiple ToF sensors on one I2C bus
# =============================================================================

import time

try:
    # gpiozero provides simple Raspberry Pi GPIO control.
    from gpiozero import OutputDevice
    GPIO_AVAILABLE = True
except ImportError:
    # On non-RPi platforms (Windows/macOS for dev), GPIO is unavailable.
    GPIO_AVAILABLE = False

# Power settle after an XSHUT pin is raised (proven timing from the
# working standalone code — must not be reduced).
POWER_DELAY = 0.125

# Every XSHUT pin registered by any ToF driver. Used so that while one
# sensor is being programmed at the factory-default address (0x29), ALL
# other sensors are held in hardware reset — otherwise the address write
# would be seen by every sensor at 0x29 and they would all end up on the
# same address, corrupting the bus.
_registered_pins = []


def register(pin):
    # Remember that this GPIO drives a ToF XSHUT line.
    if pin is not None and pin not in _registered_pins:
        _registered_pins.append(pin)


def hold_all_other_low(own_pin):
    # Lower every OTHER registered XSHUT pin into hardware reset.
    # Returns a list of (pin, OutputDevice) handles to release later.
    held = []
    if not GPIO_AVAILABLE:
        return held
    for p in _registered_pins:
        if p == own_pin:
            continue
        try:
            dev = OutputDevice(p, initial_value=False)
            held.append((p, dev))
        except Exception:
            pass
    return held


def set_all_off():
    # Turn EVERY registered XSHUT pin OFF (hardware reset for all ToF
    # sensors at once). Used at the start of init so no sensor can see
    # another's address write. Returns a list of (pin, OutputDevice)
    # handles to release later.
    held = []
    if not GPIO_AVAILABLE:
        return held
    for p in _registered_pins:
        try:
            dev = OutputDevice(p, initial_value=False)
            held.append((p, dev))
        except Exception:
            pass
    return held


def close_all(held):
    # Close OutputDevice handles WITHOUT powering the sensors back on
    # (they stay in reset). Frees the GPIO ownership so the per-sensor
    # init() can take the pins over with hold_all_other_low().
    if not GPIO_AVAILABLE:
        return
    for _, dev in held:
        try:
            dev.close()
        except Exception:
            pass


def release(held):
    # Power the held sensors back on. They keep the unique address that
    # was programmed during their own init() (addresses are retained in
    # volatile registers as long as power is applied).
    if not GPIO_AVAILABLE:
        return
    for p, dev in held:
        try:
            dev.on()
            time.sleep(POWER_DELAY)
            dev.close()
        except Exception:
            pass
