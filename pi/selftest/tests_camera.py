from .runner import SelfTestRunner, TestResult


def register_camera_tests(runner: SelfTestRunner, camera):
    # Register two camera self-tests onto the runner:
    #   1. camera_init    -- verify the camera object initialises.
    #   2. camera_capture -- capture a frame and check its resolution.
    #
    # camera -- an object implementing init() and read_raw() methods,
    #           typically from pi.hardware.camera.

    def test_camera_init():
        # Test: initialise the camera.
        # Returns SKIP if camera is None (feature disabled),
        # PASS otherwise.
        if camera is None:
            return TestResult("camera_init").skipped("Camera disabled")
        camera.init()
        return TestResult("camera_init").passed()

    def test_camera_capture():
        # Test: capture a single frame and check dimensions.
        # The frame must be at least 100x100 pixels to be usable.
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
