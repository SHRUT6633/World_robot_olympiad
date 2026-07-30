# v2.8 — Keyboard Remote Control

## What changed

I wrote `keyboard_control.py` to steer the robot with the keyboard over SSH or a direct terminal connection. Using WASD keys: W = forward, S = reverse, A = turn left, D = turn right. Space = emergency stop, Q = quit.

The robot is headless (no monitor, no keyboard) during competition—it runs on the Pi 5 which is mounted onboard. But during development, I connect to the Pi over SSH from my laptop and run `keyboard_control.py` to manually drive the robot. This is essential for testing: I can drive the robot around the course, test turning radii, check stop distances, and get a feel for the robot's handling.

## Why it changed

Before v2.8, the only way to move the robot was to write a script, flash it, and run it. This was tedious for iterative testing. I needed a way to manually drive the robot in real time to test steering response, acceleration feel, and braking behavior. The keyboard controller is basically a remote control that lets me "feel" how the robot handles.

More practically, manual driving is how we'll mark the competition course. The robot needs to drive the course once manually to record the line positions (some teams use SLAM, we use a simpler approach). The keyboard controller lets us drive the path and record waypoints for the trajectory planner.

## Errors encountered

The first implementation used Python's `curses` library for keyboard input:

```python
import curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.nodelay(True)
```

This worked over SSH. I could press W and the robot drove forward. But the motion was jerky. When I held down the W key, the terminal generated repeated characters (key repeat). Each character caused a UART message to be sent. At the default key repeat rate (~10 Hz on most systems), the robot was getting speed commands at 10 Hz. But between repeats, there were gaps where no command was sent, and the ESP32's motor controller would see the last command timeout and set speed to 0.

The timeout logic on the ESP32: if no command is received within 500 ms, the motor stops (safety feature from v2.0). When the keyboard repeats at 10 Hz, the gap between repeats is 100 ms, which is well within the 500 ms timeout. So the robot shouldn't stop. But it did. Why?

The issue is that `curses` in nodelay mode returns -1 (no key) between key repeats when the key repeat is slow enough. But actually, the real problem was different: the `curses` event-driven approach reads one character per call, and if the terminal repeats the key, it gets queued. But if there's any network latency over SSH, the characters might arrive in bursts, causing gaps longer than 500 ms.

Wait, that's not right either. The actual bug was more subtle. The `curses.getch()` call in nodelay mode returns -1 immediately if no key is pressed. My initial code was:

```python
while True:
    key = stdscr.getch()
    if key == ord('w'):
        send_drive(50, 0)
    elif key == ord('s'):
        send_drive(-30, 0)
    ...
    time.sleep(0.05)
```

When a key is held down, the terminal sends the character repeatedly at the key repeat rate. But between repeats, `getch()` returns -1. So for 90 ms out of every 100 ms (at 10 Hz repeat), the code sees no key and does nothing. The last speed command is 90 ms old, and the next one is 10 ms away. But the 90 ms gap isn't the problem—the 500 ms timeout should cover it.

The actual problem: I had forgotten to add the safety timeout on the ESP32 for the keyboard mode. Oh wait, there was a different bug. Let me think again...

Actually, the real issue I hit was that on Windows SSH clients (I use Windows Terminal + OpenSSH), the key repeat is handled differently. The SSH server on the Pi (OpenSSH) processes the key repeat and sends individual characters to the PTY, but with varying timing. Sometimes the characters arrive in a cluster (all at once after a network batch), and sometimes with large gaps. The variance in arrival timing meant that occasionally, a gap exceeded 500 ms, triggering the safety stop.

The fix was to poll the current keyboard state directly instead of relying on event-driven characters. I used the `pynput` library to track which keys are currently pressed, and send commands based on the current state, not individual events:

```python
from pynput import keyboard

current_keys = set()

def on_press(key):
    current_keys.add(key)

def on_release(key):
    current_keys.discard(key)
```

Then in the main loop, I read `current_keys` at 50 Hz and send the appropriate command:

```python
while running:
    if ord('w') in current_keys:
        send_drive(50, 0)
    elif ord('s') in current_keys:
        send_drive(-30, 0)
    else:
        send_drive(0, 0)
    if ord('a') in current_keys:
        send_steer(30)
    elif ord('d') in current_keys:
        send_steer(-30)
    else:
        send_steer(0)
    time.sleep(0.02)
```

This completely eliminated the jerky motion. The commands are sent at a steady 50 Hz regardless of key repeat rate, and the current state is always accurate.

## Alternative approaches considered

1. **Gamepad/joystick**: Use a USB gamepad instead of keyboard. Analog sticks give proportional control (50% joystick deflection = 50% speed), which is much more natural than digital WASD keys. I have an Xbox controller that works on Linux. But the robot room is crowded and carrying a controller around is annoying.

2. **Web-based control**: Run a Flask web server on the Pi and control the robot from a phone browser. This would be wireless and work with any device. But it adds network latency (Wi-Fi) and complexity (HTTP server, WebSockets).

3. **ROS 2 teleop**: Use `teleop_twist_keyboard` from ROS 2, which is a well-tested keyboard teleoperation node. But we don't have ROS 2 installed on the Pi, and installing it just for teleop is overkill.

4. **BT remote**: Connect a Bluetooth gamepad or keyboard directly to the Pi. Bluetooth adds pairing complexity and latency.

I stuck with the keyboard polling approach because it's simple, works over SSH (no extra hardware), and the pynput library is a single pip install away.

## Reasoning

The keyboard controller revealed a performance issue: the main loop runs at 50 Hz, but each loop iteration sends two UART commands (one for drive, one for steer). At 115200 baud, each command takes about 2 ms to transmit. Two commands = 4 ms = 20% of the loop time. This is fine for 50 Hz but wouldn't work at higher frequencies.

I combined the drive and steer commands into a single message:
```python
{"cmd": "drive", "speed": 50, "steer": 30}
```

The ESP32 parses the message once and sets both motor and servo. This halves the UART bandwidth.

I also added a "turbo" mode (hold Shift+W for 100% speed instead of 50%) and a "slow" mode (hold Ctrl+W for 20% speed for precise maneuvering). The speed mapping is: normal = 50%, slow = 20%, turbo = 100%. This is useful for different test scenarios.

The keyboard controller now logs all commands to a file (`.keyboard_log.csv`) for replay later. If I drive a successful course manually, I can replay the log as a trajectory in v2.5's trajectory planner.
