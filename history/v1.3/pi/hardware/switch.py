try:
    from gpiozero import Button
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

import threading
import time
from ..system.logger import log


class StartSwitch:
    def __init__(self, pin=25, pull_up=True):
        self._gpio = GPIO_AVAILABLE
        self._pin = pin
        self._pressed = threading.Event()
        self._button = None

        if self._gpio:
            self._button = Button(
                pin,
                pull_up=pull_up,
                bounce_time=0.05,
                hold_time=0
            )
            self._button.when_pressed = self._on_press

    def _on_press(self):
        self._pressed.set()
        log.info("START SWITCH PRESSED")

    def wait_for_press(self, timeout=None):
        if not self._gpio:
            log.info("Switch mock: auto-press in 2s")
            time.sleep(2)
            return True
        return self._pressed.wait(timeout=timeout)

    def was_pressed(self):
        return self._pressed.is_set()

    def reset(self):
        self._pressed.clear()

    def close(self):
        if self._button:
            self._button.close()
