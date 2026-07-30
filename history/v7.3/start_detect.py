import time
import logging


SAMPLE_INTERVAL_S = 0.01
DEBOUNCE_SAMPLES = 5
CAMERA_CONFIDENCE_FRAMES = 15
CAMERA_FRAME_INTERVAL_S = 1.0 / 15.0


class StartDetector:
    MODE_BUTTON = "button"
    MODE_CAMERA = "camera"
    MODE_DUAL = "dual"

    def __init__(self, mode=MODE_DUAL, trigger_level=0):
        self.mode = mode
        self.trigger_level = trigger_level

        self._button_started = False
        self._camera_started = False
        self._start_time = 0.0
        self._started = False

        self._last_gpio_sample = 0.0
        self._last_raw_level = 1
        self._stable_count = 0
        self._gpio_high_count = 0

        self._camera_marker_visible = False
        self._camera_confidence = 0
        self._last_camera_time = 0.0

        self.logger = logging.getLogger(self.__class__.__name__)

    def process_gpio_sample(self, raw_level):
        if self._started:
            return True

        now = time.monotonic()
        if now - self._last_gpio_sample < SAMPLE_INTERVAL_S:
            return False
        self._last_gpio_sample = now

        if raw_level == self._last_raw_level:
            self._stable_count += 1
        else:
            self._stable_count = 0
            self._last_raw_level = raw_level

        if self._stable_count >= DEBOUNCE_SAMPLES:
            if raw_level == self.trigger_level:
                return self._trigger()
            self._stable_count = 0

        return False

    def process_camera_frame(self, marker_detected):
        if self._started:
            return True

        now = time.monotonic()
        if now - self._last_camera_time < CAMERA_FRAME_INTERVAL_S:
            return False
        self._last_camera_time = now

        if marker_detected:
            self._camera_confidence += 1
        else:
            self._camera_confidence = 0

        if self._camera_confidence >= CAMERA_CONFIDENCE_FRAMES:
            return self._trigger()

        return False

    def is_button_started(self):
        return self._button_started

    def is_camera_started(self):
        return self._camera_started

    def is_started(self):
        return self._started

    def get_start_time(self):
        return self._start_time

    def get_start_source(self):
        if self._button_started:
            return "button"
        if self._camera_started:
            return "camera"
        return None

    def _trigger(self):
        self._started = True
        self._start_time = time.time()
        source = "unknown"
        if self._button_started:
            source = "button"
        elif self._camera_started:
            source = "camera"
        self.logger.info(f"Start signal detected via {source}")
        return True

    def reset(self):
        self._button_started = False
        self._camera_started = False
        self._start_time = 0.0
        self._started = False
        self._last_gpio_sample = 0.0
        self._last_raw_level = 1
        self._stable_count = 0
        self._camera_confidence = 0
        self._last_camera_time = 0.0


class HardwareButton:
    def __init__(self, gpio_pin, pull_up=True):
        self.gpio_pin = gpio_pin
        self.pull_up = pull_up
        self._simulated_level = 1

    def read(self):
        return self._simulated_level

    def simulate_press(self):
        self._simulated_level = 0

    def simulate_release(self):
        self._simulated_level = 1


class CameraMarkerDetector:
    def __init__(self):
        self._markers_detected = False

    def detect(self):
        return self._markers_detected

    def simulate_marker(self, visible):
        self._markers_detected = visible
