from .runner import SelfTestRunner, TestResult


def register_camera_tests(runner: SelfTestRunner, camera):
    def test_camera_init():
        if camera is None:
            return TestResult("camera_init").skipped("Camera disabled")
        camera.init()
        return TestResult("camera_init").passed()

    def test_camera_capture():
        if camera is None or camera._cap is None:
            return TestResult("camera_capture").skipped("Camera not initialized")
        frame = camera.read_raw()
        if frame is None:
            return TestResult("camera_capture").failed("No frame captured")
        h, w = frame.shape[:2]
        if w < 100 or h < 100:
            return TestResult("camera_capture").failed(f"Frame too small: {w}x{h}")
        return TestResult("camera_capture").passed(f"{w}x{h}", data=frame)

    runner.add("camera_init", test_camera_init)
    runner.add("camera_capture", test_camera_capture)
