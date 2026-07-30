try:
    from gpiozero import LED
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

from ..system.logger import log


class StatusLED:
    GREEN = "green"
    RED = "red"
    BLUE = "blue"

    def __init__(self, green_pin=23, red_pin=24, blue_pin=None):
        self._gpio = GPIO_AVAILABLE
        self._mock = {}
        if self._gpio:
            self._green = LED(green_pin)
            self._red = LED(red_pin)
            if blue_pin:
                self._blue = LED(blue_pin)
        else:
            self._green = None
            self._red = None
            self._blue = None
        self._patterns = {
            "off": (0, 0, 0),
            "green": (1, 0, 0),
            "red": (0, 1, 0),
            "amber": (1, 1, 0),
            "blue": (0, 0, 1),
            "green_blink": "blink_green",
            "red_blink": "blink_red",
        }
        self._blinking = False
        self._current_mode = "off"

    def set(self, color):
        self._blinking = False
        self._current_mode = color
        pattern = self._patterns.get(color, (0, 0, 0))
        if isinstance(pattern, tuple):
            self._write(*pattern)

    def _write(self, g, r, b=0):
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
        for _ in range(3):
            self.set("green")
            import time; time.sleep(0.15)
            self.set("off")
            import time; time.sleep(0.1)
        self.set("green")

    def error_sequence(self):
        self.blink("red", interval=0.3)

    def stop_blink(self):
        self._blinking = False
        if hasattr(self, "_blink_stop"):
            self._blink_stop.set()

    def close(self):
        self.stop_blink()
        self.set("off")
