# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/hardware/led.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Status LED controller
# =============================================================================

try:
    from gpiozero import LED
    GPIO_AVAILABLE = True
except ImportError:
    # Fallback when not running on a Raspberry Pi (e.g., development on PC)
    GPIO_AVAILABLE = False

from ..system.logger import log


class StatusLED:
    # Manages an RGB LED (or individual green/red LEDs) for robot status indication.
    #
    # Color patterns:
    #   "off"       – all LEDs off
    #   "green"     – solid green (normal operation)
    #   "red"       – solid red (error/fault)
    #   "amber"     – green+red simultaneously (warning/starting up)
    #   "blue"      – solid blue (special mode, e.g., manual override)
    #   "green_blink", "red_blink" – blinking variants
    #
    # Led pins default to GPIO 23 (green), 24 (red), with optional blue on another pin.
    # When GPIO is unavailable, the class falls back to logging LED state changes.

    GREEN = "green"
    RED = "red"
    BLUE = "blue"

    def __init__(self, green_pin=23, red_pin=24, blue_pin=None):
        self._gpio = GPIO_AVAILABLE
        self._mock = {}  # Reserved for future mock state tracking
        if self._gpio:
            self._green = LED(green_pin)
            self._red = LED(red_pin)
            if blue_pin:
                self._blue = LED(blue_pin)
        else:
            self._green = None
            self._red = None
            self._blue = None
        # Mapping from color name to (g, r, b) tuple or blink action string
        self._patterns = {
            "off": (0, 0, 0),
            "green": (1, 0, 0),
            "red": (0, 1, 0),
            "amber": (1, 1, 0),
            "blue": (0, 0, 1),
            "green_blink": "blink_green",
            "red_blink": "blink_red",
        }
        self._blinking = False        # True when a blink thread is active
        self._current_mode = "off"    # Current color/mode label

    def set(self, color):
        # Set LEDs to a solid color immediately. Stops any active blinking.
        self._blinking = False
        self._current_mode = color
        pattern = self._patterns.get(color, (0, 0, 0))
        if isinstance(pattern, tuple):
            self._write(*pattern)

    def _write(self, g, r, b=0):
        # Write boolean values (0 or 1) to the physical LED GPIO pins.
        # In mock mode, logs the state instead.
        if self._gpio:
            if self._green:
                self._green.value = g
            if self._red:
                self._red.value = r
            if hasattr(self, "_blue") and self._blue:
                self._blue.value = b
        else:
            names = []
            if g: names.append("GREEN")
            if r: names.append("RED")
            if b: names.append("BLUE")
            log.debug(f"LED: {'+'.join(names) if names else 'OFF'}")

    def blink(self, color, interval=0.5, count=None):
        # Blink a single color on and off in a background daemon thread.
        # interval: seconds between toggles (on/off cycle = 2 * interval)
        # count: if set, blink N times then stop (not implemented yet — loops forever until stop_blink)
        self._blinking = True
        self._current_mode = f"blink_{color}"
        import threading
        self._blink_stop = threading.Event()

        def _blink():
            while not self._blink_stop.is_set():
                self.set(color)
                if self._blink_stop.wait(interval):
                    break
                self.set("off")
                if self._blink_stop.wait(interval):
                    break
        t = threading.Thread(target=_blink, daemon=True)
        t.start()

    def blink_pattern(self, pattern, interval=0.2):
        # Blink a sequence of colors in order, cycling until stopped.
        # pattern: list of color names, e.g., ["red", "green", "blue"]
        self._blinking = True
        import threading
        self._blink_stop = threading.Event()

        def _run():
            while not self._blink_stop.is_set():
                for color in pattern:
                    if self._blink_stop.is_set():
                        break
                    self.set(color)
                    if self._blink_stop.wait(interval):
                        break
        t = threading.Thread(target=_run, daemon=True)
        t.start()

    def success_sequence(self):
        # Quick triple green flash, ending solid green.
        for _ in range(3):
            self.set("green")
            import time; time.sleep(0.15)
            self.set("off")
            import time; time.sleep(0.1)
        self.set("green")

    def error_sequence(self):
        # Continuous red blinking to indicate an error state.
        self.blink("red", interval=0.3)

    def stop_blink(self):
        # Signal all blink threads to stop.
        self._blinking = False
        if hasattr(self, "_blink_stop"):
            self._blink_stop.set()

    def close(self):
        # Clean up: stop blinking and turn off all LEDs.
        self.stop_blink()
        self.set("off")
