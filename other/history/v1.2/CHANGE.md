# v1.2 — Camera Capture Test

## Testing the PiCamera Module

The Raspberry Pi Camera Module v3 is our primary visual sensor for WRO 2026. It provides a 12-megapixel Sony IMX708 sensor with a wide 120-degree field of view, connected via the CSI (Camera Serial Interface) ribbon cable. For line following and obstacle detection, we need a reliable, high-frame-rate video stream at a resolution that balances detail against processing speed.

Our first camera test had a single goal: capture one frame and save it to disk. This would confirm that the camera is physically connected, the PiCamera2 library can access it, and the image sensor is producing valid pixel data. We used the Picamera2 library (the successor to picamera for the Raspberry Pi's libcamera-based camera stack) with OpenCV's BGR888 format, which gives us 8 bits per channel in blue-green-red order, compatible with OpenCV's default color ordering.

## First Error: Black Frames

The first frame we captured was completely black — all pixel values were near zero. The image looked like the sensor was not receiving any light, even though the camera lens was uncovered and pointing at a well-lit room. We verified that the camera was not physically blocked, the lens cap was removed, and the CSI cable was fully seated. The issue was not hardware but timing.

The Raspberry Pi Camera Module v3, like all modern CMOS image sensors, requires a stabilization period after power-up before it can produce valid frames. During this period, the sensor's automatic gain control (AGC) and automatic white balance (AWB) algorithms converge to the correct settings. If we read a frame immediately after `camera.start()`, the sensor's analog gain and exposure time have not yet settled, resulting in an underexposed (black) image.

The fix was to add a 2-second delay between `camera.start()` and `camera.capture_array()`:

```python
cam.start()
time.sleep(2)  # Critical: sensor warm-up
frame = cam.capture_array()
```

This 2-second delay allows the AGC algorithm to measure the scene brightness and adjust the analog gain and exposure time accordingly. The sensor's automatic exposure control typically converges within 1-2 seconds under normal lighting conditions (500+ lux). In darker environments, convergence can take longer because the sensor needs to accumulate enough photoelectrons to make a statistically significant measurement.

## Alternative: Using start_preview()

The Picamera2 library provides a `start_preview()` method that renders the camera feed to an XWindowing window on the desktop. This is useful for visual debugging when a monitor is connected. However, the robot does not have a display, so `start_preview()` would fail with a "No X server" error when running headless. We could use `start_preview(Preview.NULL)` to enable the preview pipeline without a display, but this adds unnecessary overhead. The preview pipeline requires the GPU to process the frame for display, consuming memory bandwidth that could be used for OpenCV processing.

## Resolution Decision: 640x480

We tested two resolutions: 640x480 and 1280x720. The PiCamera v3 sensor has a maximum resolution of 4608x2592 (12MP), but the maximum frame rate decreases with resolution. At 640x480, the camera can deliver 60 frames per second, which is more than enough for our line-following algorithm (we process at 30fps). At 1280x720, the maximum drops to 30fps, which is still viable but leaves less headroom for processing overhead.

We chose 640x480 for several reasons. First, the line-following algorithm uses color thresholding in HSV space, which requires only per-pixel color comparisons. The resolution only needs to capture the line width (typically 20-30 pixels across at 640x480 from 15cm height). Higher resolution does not improve line detection accuracy; it only increases processing time. Second, lower resolution reduces memory bandwidth. A 640x480 BGR888 frame is 921KB, while 1280x720 is 2.76MB. Processing 2.76MB at 30fps requires 82.8MB/s memory bandwidth just for frame transfer, before any OpenCV operations. Third, the Pi's VideoCore GPU can handle 640x480 scaling more efficiently, leaving the CPU cores free for control logic.

## Format: BGR888

We selected BGR888 (8 bits per channel, blue-green-red order) as the pixel format. This is OpenCV's native format, meaning we can use the captured array directly with OpenCV functions without conversion. The alternative format options include:

- RGB888: Same as BGR888 but with red-blue swapped. OpenCV would need `cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)` conversion.
- YUV420: Common for video encoding but requires conversion to RGB for processing.
- JPEG: Compressed, but decompression adds latency and quality loss for color thresholding.

BGR888 strikes the right balance between quality and direct usability. Each pixel is 3 bytes, which is larger than YUV420's 1.5 bytes per pixel, but the convenience of skipping conversion is worth the extra memory.

## Scene Requirements

Our first test frame was captured in a laboratory with fluorescent lighting (approximately 800 lux). The image was well-exposed and showed the test track clearly. We repeated the test in dimmer conditions (200 lux) and found that the image became noisy at higher analog gains. The camera's automatic gain control increases the ISO (analog gain) in low light to maintain exposure, but this amplifies sensor read noise. For reliable line following, we need at least 500 lux on the track surface. We will bring portable LED lighting to the competition venue to ensure consistent lighting.

## Learned: AGC and AWB Convergence

The main lesson from this test is that CMOS image sensors require time to stabilize after power-up. The AGC algorithm measures the scene's average luminance and adjusts the exposure time and analog gain to achieve a target brightness level. This process takes multiple frame periods because the sensor needs to read a frame, measure its brightness, adjust settings, and read again. The AWB algorithm similarly needs multiple frames to estimate the illuminant color temperature and adjust the red and blue channel gains.

For our robot, the camera is powered on once at boot and remains active throughout the race. The 2-second warm-up only happens once, at startup. After that, frames are delivered at 30fps with stable exposure. The warm-up delay was added to the boot sequence (v1.8), so by the time the robot reaches the starting line, the camera is already stabilized.

We also learned to check the camera's sensor temperature. After running for 30 minutes, the sensor temperature rose to about 45°C, which is within the operating range but causes increased dark current noise. For competition, we should keep the robot running for 5 minutes before the race to let the camera stabilize thermally.

## Frame Quality Validation

Beyond just capturing a frame, we implemented a simple quality check: the frame must have a mean pixel value between 10 and 250 (out of 255) and a standard deviation greater than 5. These thresholds catch three failure modes: black frames (mean below 10, caused by camera not warmed up), overexposed frames (mean above 250, caused by pointing at a bright light source), and uniform frames (low standard deviation, caused by lens cap still on). The quality check runs as part of the self-test in v1.8 and prints diagnostic information if the frame fails any threshold. During our testing, approximately 1 in 100 frames failed the quality check due to random sensor noise, so we retry the capture up to 3 times with 100ms delays between attempts.

## Comparison with USB Cameras

We briefly considered using a USB camera instead of the CSI PiCamera v3. USB cameras have the advantage of being hot-swappable and not requiring the delicate CSI ribbon cable. However, USB cameras on the Raspberry Pi have higher latency because the video frames must go through the USB controller and the CPU's memory management unit before reaching the application. The CSI interface connects directly to the GPU's VideoCore pipeline, bypassing the CPU entirely for the frame transfer. Benchmarks show that CSI cameras can achieve 60fps at 640x480 with less than 10ms of latency, while USB cameras at the same resolution typically achieve 30fps with 30-50ms of latency. For a line-following robot that needs to react to curves within 100ms, the lower latency of the CSI camera is a significant advantage. We also considered the Raspberry Pi Camera Module v2 (8MP Sony IMX219), but the v3's wider 120-degree field of view gives us more peripheral vision for detecting incoming obstacles.
