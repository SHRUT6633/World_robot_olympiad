# Version 1.0 — The First Brick

## What Was On My Mind

I sat down with a blank screen and a rulebook. The robot needs to:
- Drive 3 laps around a track
- Avoid red and green pillars
- Park between magenta markers
- Do it all in under 3 minutes

That is a LOT for one program. So I split it into smaller programs,
each one doing one thing. Like building with LEGO: you make one piece,
then another, then connect them.

## Why Two Computers?

The rulebook says you can use any microcontroller or single-board computer.
I had a Raspberry Pi 4 and an ESP32-S3.

**Option A:** Use only the Pi. It is powerful enough. But the Pi runs
Linux, which is not "real-time". If Linux decides to update something
while the robot is driving, the motors glitch.

**Option B:** Use only the ESP32. Real-time, no Linux delays. But the
ESP32 cannot run OpenCV (camera processing) fast enough. It would see
2 frames per second.

**Option C (what I chose):** Pi for thinking (camera, path planning,
sensor fusion). ESP32 for moving (motor PWM, servo PWM, safety checks).
They talk over UART serial at 115200 baud.

Why this? Because the Pi handles the complex stuff and the ESP32 handles
the time-critical stuff. Separation of concerns. If the Pi crashes, the
ESP32 safely stops the motors (we added a 500ms timeout for this).

## Why One Motor?

Rule 11.3 says: maximum one steering actuator (one servo).
Rule 11.5 says: no electronic differential.
Rule 11.13 says: maximum 2 driving motors, but if more than one they
must be mechanically linked.

I wanted to keep it simple. One motor, one servo, mechanical linkage
to all four wheels. Less wiring, less weight, less to break.

The downside: one motor means the same speed to all wheels. No torque
vectorring. No fancy drifting. But the rules do not need drifting.
They need reliability.

## Why Python on the Pi?

C++ would be faster. But I can write Python 5x faster than C++.
And for a competition, development speed matters more than execution
speed. The Pi 4 is fast enough for 640×480 camera at 60fps with Python.

## Why C on the ESP32?

Because ESP-IDF (the framework for ESP32) is C. C gives us direct
control over PWM timers, GPIO pins, and watchdog timers. For motor
control, you want that control.

## What I Wish I Knew Then

- I2C is noisy on long wires. Keep them short (<20cm).
- The MPU6050 accelerometer needs a few seconds to stabilize.
- The camera needs good lighting. Indoors is not enough.
- 115200 baud works but you cannot send too much data. Every packet
  is at most 32 bytes.
- I forgot to add comments. Future me would be angry.

## Files in This Version

| Folder | What It Does |
|--------|-------------|
| pi/sensors/ | Reads camera, ToF, IMU, magnetometer |
| pi/fusion/ | Combines all sensors to guess where we are |
| pi/perception/ | Tries to see pillars, lanes, walls |
| pi/control/ | Decides steering and speed |
| pi/mission/ | Decides what to do next (drive, avoid, park) |
| esp/main/ | Motor driver, servo driver, UART receiver |
| config/ | Pin numbers, I2C addresses, tuning numbers |

Everything was written without knowing if it would work. That is the
scary part of building a robot: you only know if it works when you
put it on the floor and press start.
