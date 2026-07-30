import pytest
import time
from start_detect import (
    StartDetector, HardwareButton, CameraMarkerDetector,
    SAMPLE_INTERVAL_S, DEBOUNCE_SAMPLES, CAMERA_CONFIDENCE_FRAMES,
)


class TestStartDetector:
    def test_initial_state(self):
        sd = StartDetector()
        assert not sd.is_started()
        assert sd.get_start_source() is None

    def test_gpio_debounce_requires_stable_samples(self):
        sd = StartDetector(mode=StartDetector.MODE_BUTTON)
        for _ in range(DEBOUNCE_SAMPLES - 1):
            result = sd.process_gpio_sample(0)
            assert not result
        result = sd.process_gpio_sample(0)
        assert result
        assert sd.is_started()

    def test_bouncing_gpio_resets_counter(self):
        sd = StartDetector(mode=StartDetector.MODE_BUTTON)
        sd.process_gpio_sample(0)
        sd.process_gpio_sample(0)
        sd.process_gpio_sample(1)
        sd.process_gpio_sample(0)
        for _ in range(DEBOUNCE_SAMPLES):
            sd.process_gpio_sample(0)
        assert sd.is_started()

    def test_bouncing_gpio_prevents_premature_trigger(self):
        sd = StartDetector(mode=StartDetector.MODE_BUTTON)
        samples = [0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0]
        triggered = False
        for s in samples:
            result = sd.process_gpio_sample(s)
            if result:
                triggered = True
        assert not triggered

    def test_camera_requires_confidence_frames(self):
        sd = StartDetector(mode=StartDetector.MODE_CAMERA)
        for _ in range(CAMERA_CONFIDENCE_FRAMES - 1):
            result = sd.process_camera_frame(True)
            assert not result
            if _ < CAMERA_CONFIDENCE_FRAMES - 2:
                time.sleep(0.01)
        result = sd.process_camera_frame(True)
        assert result
        assert sd.is_started()
        assert sd.is_camera_started()

    def test_camera_frame_gap_resets_confidence(self):
        sd = StartDetector(mode=StartDetector.MODE_CAMERA)
        for _ in range(CAMERA_CONFIDENCE_FRAMES - 2):
            sd.process_camera_frame(True)
            time.sleep(0.01)
        sd.process_camera_frame(False)
        assert sd._camera_confidence == 0
        for _ in range(CAMERA_CONFIDENCE_FRAMES):
            sd.process_camera_frame(True)
            time.sleep(0.01)
        assert sd.is_started()

    def test_dual_mode_either_source_triggers(self):
        sd = StartDetector(mode=StartDetector.MODE_DUAL)
        for _ in range(DEBOUNCE_SAMPLES):
            sd.process_gpio_sample(0)
        assert sd.is_started()

    def test_reset_clears_state(self):
        sd = StartDetector(mode=StartDetector.MODE_BUTTON)
        for _ in range(DEBOUNCE_SAMPLES):
            sd.process_gpio_sample(0)
        assert sd.is_started()
        sd.reset()
        assert not sd.is_started()
        assert sd.get_start_source() is None

    def test_hardware_button_simulated(self):
        btn = HardwareButton(15)
        assert btn.read() == 1
        btn.simulate_press()
        assert btn.read() == 0
        btn.simulate_release()
        assert btn.read() == 1

    def test_camera_marker_simulated(self):
        cam = CameraMarkerDetector()
        assert not cam.detect()
        cam.simulate_marker(True)
        assert cam.detect()
        cam.simulate_marker(False)
        assert not cam.detect()

    def test_start_once_only(self):
        sd = StartDetector(mode=StartDetector.MODE_BUTTON)
        for _ in range(DEBOUNCE_SAMPLES):
            sd.process_gpio_sample(0)
        assert sd.is_started()
        sd._stable_count = 0
        for _ in range(DEBOUNCE_SAMPLES * 2):
            sd.process_gpio_sample(0)
        assert sd.get_start_time() > 0
