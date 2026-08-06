# v1.8 — Startup Self-Test

## Combining All Hardware Tests

With individual hardware tests passing (I2C in v1.1, camera in v1.2, motor in v1.3, servo in v1.4, UART in v1.5, multi-sensor in v1.6, GPIO in v1.7), the next step was to combine them into a single automated self-test that runs at robot startup. The self-test must verify that every hardware component is present and functioning before the robot can begin a race.

The test sequence is:

1. GPIO LEDs: Blink green LED to confirm GPIO output works
2. I2C bus: Scan for known sensors (MPU6050, QMC5883L, VL53L0X x2, VL53L1X)
3. Camera: Capture one frame and verify it is not all black
4. Servo: Sweep through range, verify pulse generation (no jitter)
5. Motor: Brief forward/reverse test (wheels off ground)
6. Switch: Wait for start switch press (also tests input debouncing)

If any test fails, the red LED blinks rapidly and the robot enters a fault state, printing diagnostic information to the console. The race cannot begin until all tests pass.

## Error: Camera Test Takes Too Long

The first version of the self-test ran all tests sequentially, and the total boot time was approximately 12 seconds. Of that, the camera test consumed 3 seconds (2 seconds warm-up + 1 second capture and verify). For a competition robot, 12 seconds of boot time is acceptable — the robot is turned on once and runs for the entire competition day. However, we wanted to minimize boot time because the robot might need to restart between races if a software crash occurs.

The camera warm-up time is unavoidable: the sensor needs 2 seconds for AGC and AWB convergence (discovered in v1.2). However, we can overlap the camera warm-up with other tests. While the camera is warming up, we can run the I2C scan, GPIO test, and switch detection.

## The Fix: Parallel Test Execution with Threading

We restructured the self-test to run the camera test in a separate thread, allowing the I2C and GPIO tests to execute concurrently. The self-test now looks like this:

```python
import threading

camera_result = {"pass": False}

def test_camera():
    time.sleep(2)  # Warm-up
    cam = Picamera2()
    cam.configure(...)
    cam.start()
    time.sleep(0.5)
    frame = cam.capture_array()
    camera_result["pass"] = not (frame.mean() < 5)
    cam.stop()

camera_thread = threading.Thread(target=test_camera)
camera_thread.start()

# Run other tests while camera warms up
test_gpio()    # ~2 seconds (LED blinking)
test_i2c()     # ~1 second (scanning)
test_servo()   # ~3 seconds (sweep)
test_motor()   # ~1 second (brief test)

# Wait for camera thread
camera_thread.join()
```

This reduces the effective boot time from 12 seconds to approximately 9 seconds, because the camera test's 2-second warm-up overlaps with the GPIO and I2C tests. The 9-second boot time is acceptable and leaves room for further optimization.

## Alternative: Skip Camera Test in Boot

We considered making the camera test optional during boot. The argument was that the camera is unlikely to fail between boot and race start (a 5-minute window), and skipping the test would save 3 seconds of boot time. However, we decided against this because a camera failure during the race would be catastrophic — the robot cannot follow the line without vision. The 3 seconds of test time is a small investment for confidence that the camera is working.

Furthermore, the camera test is not just a presence check; it verifies that the image sensor is producing valid data (not black, not all-white, not random noise). This requires capturing and analyzing a frame, which inherently takes time. The parallelization fix reduces the impact of this delay.

## Design Decision: Self-Test is Mandatory

We debated whether the self-test could be bypassed by holding a specific GPIO pin high during boot (a "skip self-test" jumper). The argument for bypassing was speed: if we know the hardware is working (e.g., after a power cycle during a race), we might want a quick restart. We decided against this for safety reasons.

The self-test is the only mechanism that checks all hardware components before the robot moves. If a motor wire came loose during handling, the self-test would catch it before the robot starts moving, preventing a potentially dangerous situation (robot spinning in circles due to one-sided drive). The 9-second boot time is a small price to pay for this safety check. In the competition, the robot is turned on during the setup phase, well before the race starts.

## Implementation: boot.py

The self-test is implemented in `boot.py` at the project root. This script is the entry point for the robot software, called by systemd on boot. The script imports test modules from `pi/tools/`, runs them with error handling, and prints a pass/fail summary.

Key design decisions for boot.py:

1. Import path: `sys.path.insert(0, ".")` is used to ensure that modules from `pi/` can be imported regardless of the working directory (lesson from v1.0).

2. Error handling: Each test is in its own try/except block. A failure in one test (e.g., camera) does not prevent subsequent tests (e.g., motor) from running. This gives us a complete picture of which components are working.

3. Exit code: The script exits with code 0 if all tests pass, or 1 if any test fails. This allows systemd to detect failures and trigger recovery actions (e.g., reboot and retry).

4. Logging: All test results are logged to `/var/log/wro_self_test.log` with timestamps. This log is useful for diagnosing hardware issues that occur intermittently.

## Self-Test Exit Strategy

The self-test exits with code 0 (success) or 1 (failure). We designed a systemd service that runs boot.py at startup and restarts the robot if the self-test fails (Restart=on-failure with RestartSec=10). This means if a transient hardware failure (e.g., I2C bus not ready immediately after boot) causes the self-test to fail, the robot automatically reboots and tries again. After 3 consecutive failures, systemd gives up and leaves the robot in a stopped state with the red LED blinking. The service also logs each self-test attempt to /var/log/syslog with the tag WRO_SELFTEST. We can check the log remotely via Wi-Fi to diagnose hardware issues without opening the robot chassis.

## Remote Monitoring During Self-Test

During the self-test, the ESP32's UART0 is available for debugging via USB. We connected a serial-to-USB cable to the ESP32's UART0 and logged all self-test output to a laptop for analysis. This allowed us to see the self-test progress in real time, including sensor readings and error messages. For the competition, we will not have a laptop connected to the robot during the race, but we will use the self-test during the setup phase to verify hardware. The ESP32 also flashes a Morse code pattern on the red LED if a specific sensor fails: for example, two short blinks followed by a long blink indicates the MPU6050 failed (sensor at I2C address 0x68, code 2). This Morse code system helps us quickly identify which component failed without requiring a serial connection.

## Learned: Integration Testing is Essential

The self-test revealed issues that did not appear in individual component tests. For example, the I2C scan passed in isolation (v1.1), but when run after the camera test, it sometimes failed because the camera's initialization interfered with the I2C bus timing. We fixed this by adding a 100ms delay between the camera stop and the I2C start, giving the kernel time to release the I2C bus resources.

Another integration issue: the motor test (v1.3) works fine when the robot is on blocks, but when the robot is on the ground (weight on wheels), the motor draws more current and the voltage drop causes the camera to reset. We added a capacitor bank (4x 470μF electrolytic) to the power rail to handle transient current spikes.

These integration issues highlight the importance of testing the entire system together, not just individual components. The self-test provides this system-level verification every time the robot starts.
