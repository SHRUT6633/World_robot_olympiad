# v3.6 — Camera Frame Capture

## What Changed
The Raspberry Pi Camera Module v3 (IMX708 sensor) was connected to the Raspberry Pi 4's CSI camera port. We wrote `capture_frame.py` that captures 640×480 resolution frames at 60 fps and saves them as JPEG images. The camera is mounted on the front of the robot, 120 mm above the ground, angled down at 15° so it sees the track floor about 300-400 mm ahead of the robot.

This is the entry point for all vision processing. Subsequent versions (v3.7 color detection, v3.8 blob detection) will process these frames. For now, we just need reliable, high-speed frame capture without drops or corruption.

The `picamera2` library (PiCamera2, the modern replacement for the deprecated `picamera`) is used. It provides direct access to the camera's ISP (image signal processor) and supports raw Bayer and processed RGB output. We use `picamera2.Picamera2` with a configuration of `{"size": (640, 480), "format": "RGB888"}` and frame rate set via `controls.FrameRate`.

## Why
The WRO 2026 track has colored pillars (red, blue, yellow, green) that the robot must identify and interact with. The rulebook specifies pillar diameters and colors with specific RGB values. To detect them, we need a camera image. ToF sensors (v3.4/v3.5) can detect obstacles but can't distinguish colors. Vision is the only way to know which pillar is which.

We chose 640×480 at 60 fps because:
- 640×480 is sufficient resolution for pillar detection (pillars are ~50 mm diameter at 300-500 mm distance, covering about 40×120 pixels).
- 60 fps gives 16.7 ms per frame, which is fast enough for real-time control at 0.5 m/s (the robot moves 8 mm between frames).
- Higher resolution (1280×720) would reduce frame rate and increase processing time for blob detection.

## Errors Encountered

### Capture Stalls After ~100 Frames
The first version used `capture_continuous()` (the generator-based API from picamera2). It worked perfectly for about 3 seconds (~100 frames), then stalled. The script was still running, but no new frames were produced. CPU usage stayed at 100%.

```
ERROR: Frame 97 captured OK
ERROR: Frame 98 captured OK
ERROR: Frame 99 captured OK
ERROR: Frame 100 — stalled, no output for 10 seconds
ERROR: Fatal: camera stream timeout after 5.0s
```

I initially suspected a memory leak. `capture_continuous()` returns a generator that holds references to the frame buffers. If a frame isn't consumed and released quickly, the internal buffer queue fills up and blocks the camera driver. The PiCamera2 uses a multi-threaded pipeline with a buffer pool; when all buffers are in userspace (un-released), the driver can't queue new frames for the ISP to fill.

**Fix:** Replace `capture_continuous()` with explicit `capture()` calls, each time releasing the buffer immediately. This is slightly less elegant but gives explicit control over buffer lifetime.

```python
# Broken: buffer never released
for frame in camera.capture_continuous("rgb"):
    process(frame)

# Fixed: buffer released after each capture
while running:
    frame = camera.capture_array("rgb")
    process(frame)
    # 'frame' goes out of scope here, buffer returned to pool
```

We also reduced the buffer count from 4 to 2 (via `camera.configure()` with `"buffer_count": 2`), which reduces latency but requires the processing loop to be fast. Since we're only saving JPEGs for now, not doing heavy processing, this is fine.

### JPEG Compression Too Slow At 60 fps
Saving full-quality JPEGs at 60 fps caused the write queue to grow unbounded. The filesystem write rate on the Pi's SD card is about 20 MB/s. A 640×480 JPEG at quality 95 is about 200 KB, times 60 fps = 12 MB/s. That's within the SD card's capability. But `cv2.imwrite()` and `PIL.Image.save()` both take ~15 ms per frame, so at 60 fps, we spend 90% of the time writing.

**Fix:** Move JPEG encoding and writing to a separate thread. The capture thread just copies the raw frame buffer into a queue; the writer thread pops from the queue and writes to disk. This decouples capture from storage. We use `queue.Queue(maxsize=30)` to limit memory usage.

```python
write_queue = queue.Queue(maxsize=30)

def writer_thread():
    while True:
        buf, t = write_queue.get()
        cv2.imwrite(f"capture/frame_{t:.3f}.jpg", buf)
        write_queue.task_done()

t = threading.Thread(target=writer_thread, daemon=True)
t.start()
```

### Camera Warm-up Time
The first frame after camera initialization sometimes has incorrect exposure (too bright or too dark) because the AGC (automatic gain control) hasn't converged yet.

```
WARNING: Frame 0: exposure = 5ms (too dark, robot looks black)
WARNING: Frame 1: exposure = 40ms (too bright, washed out)
WARNING: Frame 5: exposure = 20ms (acceptable)
```

**Fix:** Discard the first 5 frames (similar to the IMU fix in v3.0). Also, set manual exposure time and gain for consistent lighting on the WRO track (which has controlled lighting).

## Alternatives Considered
- **picamera (legacy)**: The old library works with older firmware. We're on Bookworm with the new camera stack (libcamera), so picamera2 is required.
- **OpenCV VideoCapture**: `cv2.VideoCapture(0)` with a USB webcam instead of the Pi Camera module. Tested with a Logitech C270. The USB camera had higher latency (~50 ms) and lower max fps (30 fps). The Pi Camera module is better in every way.
- **Raw Bayer capture**: Capturing raw 10-bit Bayer data (1872×1404) and processing on the Pi's GPU. Not needed for simple blob detection.
- **Direct V4L2**: Using `v4l2-ctl` to capture frames. More control over buffer management. Too low-level for rapid development.

## Current Status
`capture_frame.py` captures 640×480 frames at a reliable 60 fps, saves JPEGs at 30 fps (writing every other frame to save disk space), and discards the first 5 frames. The buffer stall issue is fixed. We have about 1 GB of test footage from our latest run.

Next version (v3.7): Color detection — convert the rulebook's RGB pillar colors to HSV ranges and classify each pixel.
