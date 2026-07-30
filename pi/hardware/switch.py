# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/hardware/switch.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Start switch driver
# =============================================================================

try:
    from gpiozero import Button
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

import threading
import time
from ..system.logger import log


class StartSwitch:
    # Represents a physical start/proceed button that the operator presses
    # to begin the robot's mission.
    #
    # The button is configured with:
    #   pull_up=True  – internal pull-up resistor, so button connects pin to GND when pressed.
    #   bounce_time=0.05 – 50 ms debounce.
    #   hold_time=0  – no hold detection (immediate press detection only).
    #
    # When GPIO is unavailable (development on PC), wait_for_press() auto-triggers
    # after 2 seconds to allow testing without hardware.

    def __init__(self, pin=25, pull_up=True):
        self._gpio = GPIO_AVAILABLE
        self._pin = pin
        # threading.Event used for synchronisation across threads
        self._pressed = threading.Event()
        self._button = None

        if self._gpio:
            self._button = Button(
                pin,
                pull_up=pull_up,
                bounce_time=0.05,
                hold_time=0
            )
            # when_pressed callback runs in a background thread
            self._button.when_pressed = self._on_press

    def _on_press(self):
        # Called by gpiozero's internal thread when the button is pressed.
        self._pressed.set()
        log.info("START SWITCH PRESSED")

    def wait_for_press(self, timeout=None):
        # Block until the button is pressed (or timeout seconds elapses).
        # In mock mode, auto-trigger after 2 seconds.
        # Returns True if press was detected, False on timeout.
        if not self._gpio:
            log.info("Switch mock: auto-press in 2s")
            time.sleep(2)
            return True
        return self._pressed.wait(timeout=timeout)

    def was_pressed(self):
        # Non-blocking check: has the button been pressed since last reset?
        return self._pressed.is_set()

    def reset(self):
        # Clear the press state. Call after acknowledging the press to
        # allow detection of the next press.
        self._pressed.clear()

    def close(self):
        # Release GPIO resources.
        if self._button:
            self._button.close()
