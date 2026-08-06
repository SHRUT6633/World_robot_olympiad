# v2.5 — Open-Loop Trajectory

## What changed

I wrote the first trajectory planner: `trajectory_open.py`. This sends a pre-computed sequence of speed and steering commands to the ESP32, effectively making the robot drive a predetermined path without any sensor feedback. Open-loop control. It's the robot equivalent of a music box—program a sequence of notes (commands) and let it play.

A trajectory is defined as a list of waypoints. Each waypoint specifies: speed (0-100), steering angle (-45 to +45 degrees), and duration (milliseconds). The trajectory executor on the Pi reads these waypoints and sends them to the ESP32 at the correct times.

Example trajectory for a 90-degree right turn:
```python
trajectory = [
    {"speed": 40, "steer": 0,  "duration_ms": 1000},   # drive straight
    {"speed": 30, "steer": -30, "duration_ms": 800},   # turn right
    {"speed": 40, "steer": 0,  "duration_ms": 500},    # straighten out
    {"speed": 0,  "steer": 0,  "duration_ms": 0},      # stop
]
```

I also wrote `command_scheduler.py` which handles the timing: each command runs for exactly its specified duration, then the next command starts.

## Why it changed

Open-loop trajectories are useful for testing and for fixed maneuvers that the robot runs frequently. For example, the robot might need to reposition itself on the competition field after completing a lap. Rather than relying on the line-following camera to find its way back to the start, we can just tell it to drive a specific path.

More importantly, open-loop trajectories are the foundation for closed-loop trajectories (future version). Once I have the trajectory framework working, I can add feedback from odometry and gyro to correct the path in real time.

## Errors encountered

The timing drift was the first problem I hit. My initial implementation used a counter-based approach: send command 1, wait 1000 ms, send command 2, wait 800 ms, etc. The problem is that the sending itself takes time (about 2 ms for UART transmission + 5 ms for the ESP32 to process), and Python's `time.sleep()` doesn't account for this overhead.

After 5 commands, the accumulated timing error was about 35 ms. After 20 commands (a typical course run), it would be 140 ms off. At a speed of 0.5 m/s, 140 ms translates to 7 cm of position error—enough to miss a turn.

I fixed this by switching to elapsed time: instead of counting iterations, I record the start time of the trajectory and compute which command should be active based on the elapsed time. This is immune to the cumulative overhead of sending commands.

```python
# Before (buggy): counter-based
for cmd in trajectory:
    send_command(cmd)
    time.sleep(cmd['duration_ms'] / 1000.0)

# After (fixed): elapsed time
start = time.time()
cmd_index = 0
while cmd_index < len(trajectory):
    elapsed = (time.time() - start) * 1000
    if elapsed >= cumulative_time[cmd_index]:
        send_command(trajectory[cmd_index])
        cmd_index += 1
    time.sleep(0.01)  # 100 Hz loop
```

But this introduced a new problem: the command is sent slightly after the cumulative time threshold is crossed (up to 10 ms late due to the 100 Hz polling loop). This is much smaller than the counter-based drift (which was unbounded) and is bounded to at most one loop period (10 ms).

## Alternative approaches considered

1. **Send entire trajectory upfront**: Send all waypoints to the ESP32 in a single message, and let the ESP32 execute them with its own FreeRTOS timers. This would eliminate Python timing entirely. The ESP32's timers are accurate to about 1 ms. I prototyped this: the Pi sends `{"cmd": "trajectory", "points": [[40,0,1000], [30,-30,800], ...]}`, the ESP32 stores them in an array, and a timer task steps through them. This worked well but has limited capacity—the ESP32 only has 512 KB of RAM, and a complex trajectory with 100 waypoints uses about 3 KB. Not a problem for memory, but the UART message for 100 waypoints is about 3 KB, which takes about 260 ms to transmit at 115200 baud. The Pi would be blocked for 260 ms.

2. **Use hardware timer on Pi**: The Raspberry Pi has hardware timers accessible through `/dev/timer`. But this requires root access and a custom kernel module. Too much hassle.

3. **Pre-calculate on ESP32**: Use the ESP32's RMT (Remote Control) peripheral to generate timed command sequences. The RMT is designed for exactly this kind of thing—it can output a sequence of pulses with precise timing. But it's low-level and I'd have to write a lot of code.

I stuck with the elapsed-time approach because it's simple, works within the existing architecture, and the remaining timing error (0-10 ms per command) is acceptable for open-loop control. The trajectory planner will be replaced with closed-loop control later anyway.

## Reasoning

The open-loop trajectory tests revealed something interesting: the robot's actual path differs from the calculated path by about 7% in distance and 3 degrees in heading over a 5-meter run. This is due to:
1. Tire slip during acceleration (the odometry reports more distance than actual due to wheel spin).
2. Steering servo response time (the servo takes about 100 ms to reach the commanded angle).
3. Ackermann approximation error (the four-bar linkage doesn't perfectly follow the calculated Ackermann angles).

These errors are systematic and can be calibrated out. I added a calibration mode: run a known trajectory, measure the actual end position, and compute correction factors for speed and steering. The correction factors are stored in a calibration file and applied to all future trajectories.

The trajectory system also exposed a Pi-side performance issue: the Python UART write blocks for about 2 ms per command (115200 baud, ~25 bytes per JSON message). During that 2 ms, the robot is unresponsive. I added a non-blocking send queue using `threading.Thread` and a `queue.Queue`. The main loop puts commands on the queue, the UART thread sends them from the queue. This reduces the main loop jitter from 2 ms to about 0.1 ms.
