# v1.0 — Project Skeleton

## Architecture Decision: Why Raspberry Pi + ESP32-S3?

When we started designing the WRO 2026 robot, the first and most critical decision was choosing the main computing platform. We considered three approaches: a single powerful board doing everything, two boards splitting the work, or a microcontroller-only approach. After extensive research and prototyping, we settled on a dual-board architecture: Raspberry Pi 4 for high-level processing and ESP32-S3 for low-level real-time control.

The Raspberry Pi 4 was chosen for its mature Linux ecosystem, Python support, and ability to run OpenCV for camera-based line following and object detection. With a 1.8GHz quad-core ARM Cortex-A72 and 4GB of RAM, it provides ample compute for running Python, processing camera frames at 30fps, and communicating with the ESP32 over UART. However, the Pi is not a real-time system. Linux introduces scheduling latency that can exceed 10ms under load, making it unsuitable for precise motor PWM generation or servo pulse timing. Additionally, the Pi's GPIO pins operate at 3.3V logic level, while many of our sensors and motor drivers require 5V or have specific timing constraints that are better handled by a microcontroller.

The ESP32-S3 complements the Pi perfectly. It has two Xtensa LX7 cores running at 240MHz, built-in Wi-Fi and Bluetooth (useful for debugging), and most importantly, it has a dedicated LEDC PWM controller that can generate servo pulses with microsecond precision. The ESP32 runs FreeRTOS, which provides deterministic task scheduling. We assigned all time-critical operations to the ESP32: motor PWM generation, servo pulse timing, I2C sensor polling at fixed intervals, and UART communication with the Pi. This offloads real-time constraints from the Pi, allowing it to focus on vision processing and strategy logic without worrying about missing a servo pulse.

## Why Python for Pi and C for ESP32?

The choice of programming language for each board was driven by their respective roles and constraints. The Raspberry Pi runs Python for several reasons. First, Python has the best library support for computer vision. OpenCV, Picamera2, NumPy, and scikit-image all have first-class Python bindings, allowing us to prototype vision algorithms quickly. Second, Python's dynamic nature allows for rapid iteration during development. We can test a new line-following algorithm by editing a single file and re-running the script, without compilation. Third, the Pi has enough memory and CPU power to handle Python's overhead. The 4GB RAM means we can load full-resolution camera frames without worrying about garbage collection pauses.

On the ESP32, we use C with the ESP-IDF framework. C gives us direct hardware access without abstraction overhead. When generating a 50Hz servo PWM signal with 14-bit resolution, we need to write to LEDC registers with precise timing. In C, we configure the timer once and let the hardware handle the rest, with zero CPU overhead. Python on a microcontroller would introduce unpredictable latency due to garbage collection and interpreter overhead. The ESP-IDF also provides well-tested drivers for UART, I2C, SPI, and GPIO, all with C APIs that map directly to hardware registers. We considered using MicroPython on the ESP32, but the limited RAM (512KB) and lack of real-time guarantees made it unsuitable for our motor control requirements.

## Directory Structure Decision

We organized the repository into two top-level directories: `pi/` for Raspberry Pi code and `esp/` for ESP32 code. This separation serves multiple purposes. First, it prevents accidental cross-contamination. A developer working on motor control should not need to navigate through Python files to find the C source. Second, the build systems are completely different. The Pi code uses standard Python with pip dependencies, while the ESP32 code uses the ESP-IDF build system based on CMake. Mixing them in a single directory would confuse both toolchains. Third, deployment is simpler: we can rsync only the `pi/` directory to the robot and flash only the `esp/` directory to the ESP32 without transferring unnecessary files.

Inside `pi/`, we have subdirectories for `sensors/` (camera wrappers, I2C sensor drivers), `vision/` (OpenCV processing pipelines), `control/` (high-level strategy and state machine), and `tools/` (testing and calibration utilities). Inside `esp/`, the standard ESP-IDF layout applies: `main/` contains the application code, `components/` contains reusable driver modules. The `main/` directory itself contains `main.c` as the entry point, plus separate modules for motor control, servo control, I2C sensor reading, and UART communication.

## Separation of High-Level Logic from Low-Level Control

We designed the system with a clear boundary between high-level logic on the Pi and low-level control on the ESP32. The Pi is responsible for vision processing, path planning, strategy decisions, and monitoring. The ESP32 is responsible for motor PWM generation, servo pulse timing, sensor polling, and executing immediate commands from the Pi.

Communication happens over UART at 115200 baud. The Pi sends structured commands like "MOTOR left=0.5 right=0.8" or "SERVO angle=-15" or "READ_SENSORS". The ESP32 parses these commands, executes them with precise timing, and sends back sensor data or acknowledgment. This protocol is deliberately simple: text-based, newline-delimited, with no binary encoding. This makes debugging trivial — we can monitor the UART with a serial terminal and see exactly what commands are being exchanged.

The advantage of this separation became clear during testing. When we changed the line-following algorithm on the Pi, we did not need to touch any ESP32 code. Conversely, when we optimized the motor PID loop on the ESP32, the Pi continued sending the same commands unchanged. This independence allows two developers to work on different layers simultaneously without merge conflicts.

## First Error: Python Import from Wrong Directory

Our first attempt to run the skeleton code ended in failure. We navigated to `history/v1.0/` and ran `python main.py`, expecting to see the startup message. Instead, we got:

```
Traceback (most recent call last):
  File "main.py", line 6, in <module>
    from sensors.camera import PiCamera
ModuleNotFoundError: No module named 'sensors'
```

The problem was clear: Python looks for modules in the current working directory, but `sensors/` is inside `pi/`, not at the top level. When we ran `main.py` directly from the `history/v1.0/` directory, Python had no way to find the `sensors` package because it was in `pi/sensors/`. The fix was to either run from the `pi/` directory or add `sys.path.insert(0, ".")` to the Python import path. We chose to document this error intentionally in the v1.0 code to remind ourselves that Python's module resolution depends on the working directory, not the script's location. This is a common pitfall that we would encounter again if we ran scripts from cron jobs or systemd services.

## Alternatives Considered

We evaluated several alternatives before settling on the Pi + ESP32 architecture.

Arduino (Uno or Mega): The Arduino ecosystem is popular for robotics, but the ATmega328P on the Uno is severely underpowered for vision processing. With only 2KB of RAM and 16MHz clock speed, it cannot run OpenCV or process camera frames. Even the Arduino Mega with 256KB RAM is insufficient. The Arduino's lack of a Linux operating system also means no networking stack, no file system, and no ability to run Python. While Arduino is excellent for simple motor control, it could not handle the vision processing required for WRO line following.

Jetson Nano: NVIDIA's Jetson Nano has a GPU capable of running neural networks and would have been ideal for advanced vision tasks. However, at $249 retail, it is significantly more expensive than the Raspberry Pi 4 ($55) plus ESP32-S3 ($8). The Jetson also draws 5-10W compared to the Pi's 3W, which would require a larger battery. For WRO 2026, where the track has colored lines and obstacles but does not require deep learning, the Jetson's GPU is unnecessary overhead. If we were doing object classification or semantic segmentation, the Jetson would be justified, but for simple line detection using color thresholding and contour finding, the Pi's CPU is sufficient.

ESP32 Only: We considered using two ESP32s — one for camera processing and one for motor control. The ESP32-S3 has a camera interface (DVP) that can capture 640x480 frames, but processing them requires significant RAM. Each 640x480 BGR888 frame is 921KB, nearly double the ESP32's available 512KB RAM. We would need to use JPEG compression (which the ESP32 camera driver supports), but then we lose pixel-level accuracy for line detection. Furthermore, OpenCV is not available for ESP32; we would need to implement Sobel edge detection, Hough transforms, and color thresholding from scratch in C. While possible, this would take months of development. The Pi with Python and OpenCV accomplishes the same task in hours.

UART Communication Protocol: We considered using I2C for Pi-to-ESP32 communication instead of UART. I2C has the advantage of supporting multiple slaves on the same bus, but it requires the Pi to act as the master and the ESP32 as the slave. This means the ESP32 cannot initiate communication (e.g., to send an alert when a sensor detects an obstacle). UART is bidirectional and simpler to debug. We also considered SPI, which offers higher bandwidth, but SPI requires four wires (MOSI, MISO, SCK, CS) plus per-device chip select lines, while UART needs only two wires (TX, RX). For our modest 115200 baud rate, UART has plenty of bandwidth for sending commands and receiving sensor data at 100Hz.

## Lessons Learned

The most important lesson from v1.0 is that architectural decisions made early have cascading effects on every subsequent phase. Choosing the Pi + ESP32 split defined our communication protocol, our development workflow, our testing strategy, and even our directory structure. We could have spent weeks trying to make a single-board solution work, but by accepting the complexity of a dual-board system, we gave ourselves the best tools for each task. The initial import error, while trivial, taught us to be careful about Python's working directory behavior — a lesson that would save us debugging time in later versions.
