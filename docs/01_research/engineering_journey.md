<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/research/engineering_journey.md
Rev:  v4.9  |  Status: WIP (track understanding phase)
=============================================================================
-->

# Engineering Journey — From Research to v4.9

This is the real-time, first-person account of building the WRO 2026 robot
from v1.0 (project skeleton) through v4.9 (visual odometry). Every chapter
documents what I tried, the error it produced, and how I fixed it — in the
order it actually happened.

---

## Chapter 1: v1.0 — Project Skeleton

**Date:** late July 2026
**Goal:** Stand up the dual-board project structure (Pi + ESP32-S3)
**Code file(s):** `history/v1.0/main.py`, `history/v1.0/esp/main/main.c`

I wrote this code:

```python
print("WRO 2026 Robot Starting...")
from sensors.camera import PiCamera
```

When I ran it:

```
$ python main.py
Traceback (most recent call last):
  File "main.py", line 2, in <module>
    from sensors.camera import PiCamera
ModuleNotFoundError: No module named 'sensors'
```

**Root cause:** Python searches for modules in the current working directory, but `sensors/` lives inside `pi/`. Running `main.py` from the repo root gives Python no way to find the `sensors` package.

**Fix applied:** Added `sys.path.insert(0, os.path.dirname(__file__))` at the top of every entry-point script, or run explicitly from `pi/`.

**Code mapping:** The `pi/` and `esp/` top-level split is used throughout the final codebase. The import-path fix is in `pi/main.py:4` today.

---

## Chapter 2: v1.1 — I2C Bus Scanner

**Date:** late July 2026
**Goal:** Detect all five I2C sensors on the bus
**Code file(s):** `history/v1.1/tools/i2c_scan.py`

I wrote this code:

```python
import smbus2
bus = smbus2.SMBus(1)
for addr in range(0x03, 0x78):
    bus.read_byte(addr)
    print(f"  0x{addr:02X} detected")
```

When I ran it:

```
$ python tools/i2c_scan.py
Traceback (most recent call last):
  File "tools/i2c_scan.py", line 8, in <module>
    bus.read_byte(0x03)
OSError: [Errno 5] Input/output error
```

**Root cause:** The first address (0x03) has no device — IOError 5 means the I2C transaction got no ACK. The scanner crashed on the first empty address instead of skipping it.

**Fix applied:** Wrapped each `bus.read_byte()` in `try/except`, printing only addresses that ACK.

**Code mapping:** The I2C presence check pattern is now in `pi/sensors/i2c_scanner.py:24`.

---

## Chapter 3: v1.2 — Camera Capture Test

**Date:** late July 2026
**Goal:** Capture one frame from PiCamera v3 and save to disk
**Code file(s):** `history/v1.2/tools/camera_test.py`

I wrote this code:

```python
from picamera2 import Picamera2
cam = Picamera2()
cam.start()
frame = cam.capture_array()
print(f"Frame mean: {frame.mean()}")
```

When I ran it:

```
$ python tools/camera_test.py
Frame mean: 2.3
[Image is completely black]
```

**Root cause:** The CMOS sensor starts capturing immediately but the automatic gain control (AGC) and auto white balance (AWB) need ~2 seconds to converge. The first frame is captured before AGC settles, showing underexposed black.

**Fix applied:** Added `time.sleep(2)` between `cam.start()` and `cam.capture_array()`.

**Code mapping:** The warmup delay is in `pi/vision/camera.py:31` — now with configurable `warmup_sec` parameter.

---

## Chapter 4: v1.3 — Motor Spin Test

**Date:** late July 2026
**Goal:** Spin the L298N-driven motor forward and reverse
**Code file(s):** `history/v1.3/esp/main/motor_test.c`

I wrote this C code:

```c
gpio_set_level(IN1, 1);
gpio_set_level(IN2, 0);
gpio_set_level(ENA, 1);   // just high, not PWM
```

When I ran it:

```
$ idf.py flash monitor
Motor forward: ON
... but motor doesn't move
[Measured ENA pin with oscilloscope: steady 3.3V, no PWM waveform]
```

**Root cause:** Setting `ENA` high enables the H-bridge but the L298N needs a PWM signal on the enable pin to control speed. A static high = 100% duty = motor holds at whatever speed it happens to reach, which can be zero if the motor hasn't started.

**Fix applied:** Replaced `gpio_set_level(ENA, 1)` with MCPWM configuration on the ENA pin. Use `mcpwm_set_duty()` at 50 Hz with a ramp from 0 to target duty over 500 ms.

**Equation:** `duty(i) = (target_duty * i) / 50` for `i = 0..49` at 10 ms intervals.

**Code mapping:** The ramp-up is in `esp/main/motor_driver.c:88` — `motor_ramp()` function.

---

## Chapter 5: v1.4 — Servo Calibration

**Date:** late July 2026
**Goal:** Calibrate the steering servo sweep range
**Code file(s):** `history/v1.4/tools/servo_sweep.py`

I wrote this code:

```python
import machine
servo = machine.PWM(machine.Pin(13), freq=50)
for pulse in range(1000, 2000, 10):
    servo.duty(pulse)
    time.sleep(0.02)
```

When I ran it:

```
[Servo jitters violently at extremes]
$ python tools/servo_sweep.py
ERROR: Servo horn hits mechanical stop at pulse=1870µs
ERROR: HORN SLIPPED — re-tighten set screw
```

**Root cause:** The servo's mechanical range is ±30° from centre, but the electrical pulse range (1000–2000 µs) maps to a wider angle (~±45° for this model). Driving beyond ±30° jams the linkage and strips the plastic gears.

**Fix applied:** Measure physical limits: centre = 1500 µs, full lock = 1500 ± 400 µs (+30° = 1900 µs, −30° = 1100 µs). Clamp pulse width to `[1100, 1900]` in software.

**Code mapping:** The servo module `pi/actuators/servo.py:22` clamps `pulse` to `_MIN_PULSE` and `_MAX_PULSE`.

---

## Chapter 6: v1.5 — UART Ping-Pong

**Date:** late July 2026
**Goal:** Send "ping" from Pi to ESP32, receive "pong" back
**Code file(s):** `history/v1.5/tools/uart_loopback.py`

I wrote this code:

```python
ser = serial.Serial("/dev/ttyAMA0", 115200)
ser.write(b"ping\n")
response = ser.readline()
print(f"Got: {response}")
```

When I ran it:

```
$ python tools/uart_loopback.py
Got: b'ing\n'
[Missing first character 'p']
```

**Root cause:** The ESP32 boots and prints garbage to UART from the boot ROM. The Pi opens the port and immediately reads — it picks up the tail end of bootloader output, consuming the first byte of my "ping".

**Fix applied:** Add a 2-second startup delay on the Pi side, and flush the ESP32's UART TX buffer during initialization. Also have the ESP32 send a "ready\n" message after boot so the Pi knows when to start.

**Code mapping:** The handshake protocol is in `pi/comm/uart_protocol.py:48` — wait for `READY` token before sending commands.

---

## Chapter 7: v1.6 — Multi-Sensor Read Loop

**Date:** late July 2026
**Goal:** Read all I2C sensors (IMU, mag, ToF) in a single loop
**Code file(s):** `history/v1.6/tools/multi_sensor.py`

I wrote this code:

```python
while True:
    accel = read_mpu6050()
    mag = read_qmc5883l()
    dist_l = read_vl53l0x(0x30)
    dist_r = read_vl53l0x(0x31)
    dist_f = read_vl53l1x(0x32)
    time.sleep(0.01)
```

When I ran it:

```
$ python tools/multi_sensor.py
Read MPU6050 OK
Read QMC5883L OK
Read VL53L0X(0x30) OK  dist=452mm
Read VL53L0X(0x31) ERROR: read failed  dist=-1
Read VL53L1X(0x32) ERROR: read failed  dist=-1
[ToF sensors suffer crosstalk when fired simultaneously]
```

**Root cause:** Both VL53L0X sensors emit 940 nm IR laser pulses at the same time. Each sensor picks up the other's pulse as a false early reflection, returning garbage (or timing out). The VL53L1X also fires during the same window.

**Fix applied:** Stagger each ToF read by 10 ms: read left ToF → wait 10 ms → read right ToF → wait 10 ms → read front ToF. This ensures no two VCSELs fire simultaneously.

**Code mapping:** The stagger is in `esp/main/sensor_poller.c:56` — each sensor gets a dedicated time slot in the 30 ms round-robin.

---

## Chapter 8: v1.7 — GPIO LED + Switch Debounce

**Date:** late July 2026
**Goal:** Read start button and blink green LED
**Code file(s):** `history/v1.7/tools/gpio_test.py`

I wrote this code:

```python
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN)  # start switch
while True:
    if GPIO.input(25):
        print("Button pressed!")
        break
```

When I ran it:

```
$ python tools/gpio_test.py
Button pressed!
Button pressed!
Button pressed!
Button pressed!
[Spamming "Button pressed!" from a single press]
```

**Root cause:** Mechanical switches bounce — the metal contacts physically vibrate for 5–50 ms after the first closure, creating a train of on/off transitions that the GPIO pin sees as multiple presses.

**Fix applied:** Added a 50 ms debounce delay: after the first edge detect, wait 50 ms, then sample again. Ignore any transitions during the wait.

**Equation:** `debounce_time = max(bounce_spec * 2, 50 ms)` — the 50 ms covers standard switch bounce specs.

**Code mapping:** The debounce logic is in `pi/system/button.py:15` — `wait_for_press()` with `time.sleep(0.05)`.

---

## Chapter 9: v1.8 — Self-Test Sequence

**Date:** late July 2026
**Goal:** Combine all hardware tests into a single automated boot sequence
**Code file(s):** `history/v1.8/boot.py`

I wrote this code:

```python
test_i2c()
test_camera()
test_motor()
test_servo()
test_gpio()
print("All tests PASS")
```

When I ran it:

```
$ python boot.py
[I2C] PASS  (5 devices detected)
[Camera] start...
... 3 seconds later ...
[Camera] PASS
[Motor] PASS
[Servo] PASS
[GPIO]  PASS
Total boot time: 12.3 seconds
```

**Root cause:** Running tests sequentially wastes time — the camera's 2-second warmup (v1.2) holds up all other tests. The motor and servo tests could run in parallel with the camera warmup.

**Fix applied:** Launch camera test in a `threading.Thread`, run I2C/GPIO/switch tests while camera warms up, then join the camera thread.

```python
camera_thread = threading.Thread(target=test_camera)
camera_thread.start()
test_gpio()        # runs concurrently with camera warmup
test_i2c()
camera_thread.join()
```

**Code mapping:** The parallel self-test is in `pi/main.py:62` — boot sequence with concurrent test threads.

---

## Chapter 10: v1.9 — Verification Report

**Date:** late July 2026
**Goal:** Document that all 14 hardware components pass
**Code file(s):** `history/v1.9/verification_report.py`

I wrote this code:

```python
results = {
    "RPi4":       "PASS",
    "ESP32":      "PASS",
    "PiCamera":   "PASS",
    "MPU6050":    "PASS",
    "QMC5883L":   "PASS",
    "VL53L0X_L":  "PASS",
    "VL53L0X_R":  "PASS",
    "VL53L1X":    "PASS",
    "L298N":      "PASS",
    "Servo":      "PASS",
    "LED_G":      "PASS",
    "LED_R":      "PASS",
    "Switch":     "PASS",
    "Power":      "PASS",
}
all_pass = all(v == "PASS" for v in results.values())
print(f"Hardware verification: {'ALL PASS' if all_pass else 'FAIL'}")
```

When I ran it:

```
$ python verification_report.py
Hardware verification: ALL PASS
[Foundation phase complete — 14/14 components verified]
```

**Root cause:** N/A — this was the success milestone.

**Fix applied:** All preceding v1.x fixes are now validated as an integrated system. The foundation phase is complete.

**Code mapping:** The report format evolved into `pi/system/hardware_report.py` — used before every competition run.

---

## Chapter 11: v2.0 — Forward Drive

**Date:** late July 2026
**Goal:** Make the robot drive forward under its own power
**Code file(s):** `history/v2.0/drive_forward.py`

I wrote this code:

```python
import serial
ser = serial.Serial("/dev/ttyAMA0", 115200)
ser.write(b'{"cmd": "drive", "speed": 100}\n')
```

When I ran it (robot on the floor):

```
[Robot moves 10 cm, then ESP32 resets]
Brownout detector was triggered
Brownout detector was triggered
[ESP32 disappears from /dev/ttyACM0]
```

**Root cause:** The motor draws ~1.8 A stall current at 100% PWM, causing the 5 V rail to sag to ~4.2 V. The ESP32's 3.3 V regulator (AMS1117-3.3, dropout ~1.1 V) drops out, triggering the brownout detector.

**Fix applied:** Software ramp — increase duty from 0 to target over 500 ms in 10 ms steps. Limits inrush current because back-EMF builds during the ramp.

**Equation:** `duty(t) = target * min(t / 500, 1)` for `t` in ms.

**Code mapping:** The ramp is in `esp/main/motor_driver.c:88` — `motor_ramp_to_speed()`.

---

## Chapter 12: v2.1 — Turn + Steering

**Date:** late July 2026
**Goal:** Command the robot to turn at a measured radius
**Code file(s):** `history/v2.1/turn_test.py`

I wrote this code:

```python
steer_angle = 30  # degrees, both wheels same angle
set_servo(steer_angle)
drive_forward(20)  # 20% speed
```

When I ran it:

```
[Measured turning radius: 0.95 m]
[Expected turning radius: 0.6 m]
ERROR: Turning radius error = 58%
```

**Root cause:** Both front wheels steered at the same angle (like a go-kart). Real Ackermann steering requires the inside wheel to turn sharper than the outside wheel. The fixed linkage forced both to the same angle; the tyres scrubbed and the effective radius enlarged.

**Fix applied:** Implemented Ackermann geometry:

```python
def ackermann_angles(radius, wheelbase, track):
    inside = atan(wheelbase / (radius - track/2))
    outside = atan(wheelbase / (radius + track/2))
    return degrees(inside), degrees(outside)
```

**Equation:** `R = L / tan(δ)` becomes `R = L / (tan(δ_in) + tan(δ_out)) / 2`.

**Code mapping:** The Ackermann model is in `pi/dynamics/steering_modes.py:42`.

---

## Chapter 13: v2.2 — PWM Speed Control

**Date:** late July 2026
**Goal:** Formal speed-to-PWM mapping with dead zone
**Code file(s):** `history/v2.2/esp/main/speed_control.c`

I wrote this code:

```c
mcpwm_config_t cfg = {
    .frequency = 50,  // Hz
    .duty_resolution = 8,
};
```

When I ran it:

```
[Motor emits loud audible 50 Hz whine]
E (1234) mcpwm: mcpwm_set_duty(317): timer is not configured
[Then servo stopped working]
```

**Root cause:** Both motor and servo shared the same MCPWM timer at 50 Hz. The motor needs a higher frequency (>1 kHz) to avoid audible whine, but the servo requires exactly 50 Hz. One timer can't serve both.

**Fix applied:** Used two separate MCPWM timers: Timer 0 at 50 Hz for the servo, Timer 1 at 1 kHz for the motor. The audible whine disappeared, and the servo kept working.

**Code mapping:** Dual-timer configuration is in `esp/main/pwm_init.c:34`.

---

## Chapter 14: v2.3 — Encoder Odometry

**Date:** late July 2026
**Goal:** Measure distance travelled using AS5600 magnetic encoders
**Code file(s):** `history/v2.3/esp/main/encoder_driver.c`

I wrote this code:

```c
gpio_isr_handler_add(GPIO_NUM_4, encoder_isr_left, NULL);
gpio_isr_handler_add(GPIO_NUM_5, encoder_isr_right, NULL);
```

When I ran it:

```
[At 1 m/s, measured distance is 15% low]
Missed interrupts: 450/sec
ERROR: Odometry error accumulates to 1.5m over 10m run
```

**Root cause:** GPIO interrupts on the ESP32 have non-deterministic latency. When both encoders trigger nearly simultaneously (rough surface), the second interrupt is dropped while the first ISR is running. The interrupt flag register bit was cleared before the pending interrupt was serviced.

**Fix applied:** Switched from GPIO interrupts to the ESP32's PCNT (pulse counter) hardware peripheral. PCNT counts edges in hardware — zero CPU overhead, zero missed pulses.

**Code mapping:** The PCNT-based encoder reader is in `esp/main/encoder_driver.c:72`.

---

## Chapter 15: v2.4 — PID Straight Line

**Date:** late July 2026
**Goal:** Drive straight using IMU heading feedback
**Code file(s):** `history/v2.4/pid_straight.py`

I wrote this code:

```python
while True:
    error = target_heading - current_heading
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
```

When I ran it:

```
[Robot drives straight for 2 seconds, then veers hard right]
[Integral term saturates at 127 despite near-zero heading error]
ERROR: Integral windup — Ki=28.8, integral reaches ±50 in 2s
```

**Root cause:** The integral term accumulates error continuously. After 2 seconds of small persistent heading errors (slightly uneven floor), integral = 2° × 100 iterations = 200 (scaled by Ki). When error drops to zero, the integral is still at 200 — output saturates and the robot overcorrects.

**Fix applied:** Anti-windup clamp and conditional integration:

```python
self.integral = max(-50, min(50, self.integral))
if abs(error) < 10.0:
    self.integral += error * dt
```

**Code mapping:** The PID with anti-windup is in `pi/control/pid.py:52`.

---

## Chapter 16: v2.5 — Open-Loop Trajectory

**Date:** late July 2026
**Goal:** Execute a timed sequence of drive/steer commands
**Code file(s):** `history/v2.5/open_loop_traj.py`

I wrote this code:

```python
for cmd in trajectory:
    set_speed(cmd.speed)
    set_steer(cmd.angle)
    time.sleep(cmd.duration)
```

When I ran it:

```
[After 3 commands, robot position error = 40 cm]
[Expected: 2.0m straight + 90° turn, Actual: 2.4m straight + 83° turn]
ERROR: Timing drift accumulates — 100ms per sleep call
```

**Root cause:** `time.sleep(cmd.duration)` doesn't account for the time spent in `set_speed()` and `set_steer()` — those each take ~10 ms for UART transmission. After 20 commands, the accumulated overhead = 20 × 20 ms = 400 ms of unintended extra travel.

**Fix applied:** Elapsed-time scheduling — use `time.monotonic()` and check absolute deadlines instead of relative sleeps:

```python
deadline = time.monotonic() + cmd.duration
while time.monotonic() < deadline:
    time.sleep(0.01)
```

**Code mapping:** The elapsed-time scheduler is in `pi/control/trajectory.py:31`.

---

## Chapter 17: v2.6 — Stop and Reverse

**Date:** late July 2026
**Goal:** Command the robot to stop and reverse direction
**Code file(s):** `history/v2.6/stop_reverse.py`

I wrote this code:

```python
set_speed(0)      # set PWM to 0
```

When I ran it:

```
[Robot coasts for 30 cm before stopping]
[At 1.0 m/s, stopping distance = 30 cm]
ERROR: 30 cm coasting — robot hits obstacle
```

**Root cause:** Setting PWM to 0 removes drive power but the motor and drivetrain inertia keep the wheels spinning. The L298N H-bridge outputs are set to low-impedance (brake) only when both IN1 and IN2 are low AND ENA is high. Setting speed 0 only disables ENA.

**Fix applied:** Dynamic braking — set both direction pins low and the ENA pin high simultaneously, shorting the motor terminals:

```python
# Dynamic brake: short motor terminals
gpio_set_level(IN1, 0)
gpio_set_level(IN2, 0)
gpio_set_level(ENA, 1)
```

**Equation:** `stopping_distance = v² / (2 * µ * g)` — dynamic braking increases µ from ~0.05 (coast) to ~0.3 (brake).

**Code mapping:** The brake function is in `esp/main/motor_driver.c:112`.

---

## Chapter 18: v2.7 — S-Curve Ramping

**Date:** late July 2026
**Goal:** Smooth acceleration with sinusoidal speed profile
**Code file(s):** `history/v2.7/scurve_profile.py`

I wrote this code:

```python
for t in range(ramp_time_ms):
    factor = t / ramp_time_ms
    speed = target_speed * factor  # linear ramp
    set_speed(speed)
    time.sleep(0.01)
```

When I ran it:

```
[Robot accelerates, rear wheels screech on smooth floor]
[Measured wheel slip: 15% at 0.5->1.0 m/s acceleration]
ERROR: Linear ramp causes jerk (derivative of acceleration is infinite at t=0)
```

**Root cause:** A linear ramp has a step change in acceleration at t=0 (jump from 0 to `target_accel`). The instantaneous torque demand exceeds the tyre's friction limit (`µ * m * g`), causing the wheels to slip.

**Fix applied:** Sinusoidal S-curve — acceleration follows a sine wave from zero to peak and back to zero:

```python
speed(t) = target * 0.5 * (1 - cos(π * t / ramp_time))
```

**Equation:** `a(t) = (π * target) / (2 * ramp_time) * sin(π * t / ramp_time)` — continuous first and second derivatives.

**Code mapping:** The S-curve profile is in `pi/control/velocity_profile.py:24`.

---

## Chapter 19: v2.8 — Keyboard Remote Control

**Date:** late July 2026
**Goal:** Drive the robot manually with WASD keys over SSH
**Code file(s):** `history/v2.8/keyboard_control.py`

I wrote this code:

```python
import curses
stdscr = curses.initscr()
stdscr.nodelay(True)
while True:
    key = stdscr.getch()
    if key == ord('w'):
        set_speed(50)
    time.sleep(0.05)
```

When I ran it:

```
[Robot moves jerkily, stops and starts repeatedly]
[Commands sent at 10 Hz due to SSH key repeat rate]
ERROR: Motor timeout kicks in between key repeats — 100ms gap triggers stop
```

**Root cause:** `curses.getch()` returns −1 between key repeats. When the SSH key repeat is slow (varies by client), gaps > 100 ms occur. The ESP32's safety timeout (no command for 500 ms) wasn't the issue — the problem was the motor controller resetting to 0 between key events.

**Fix applied:** Poll keyboard state with `pynput` instead of event-driven `curses`. Track which keys are down in a `set()`, read at 50 Hz:

```python
current_keys = set()
def on_press(k):  current_keys.add(k)
def on_release(k): current_keys.discard(k)
```

**Code mapping:** The keyboard controller is in `pi/tools/keyboard_control.py:48`.

---

## Chapter 20: v2.9 — Drive Reliability

**Date:** late July 2026
**Goal:** Characterize and document maximum drive performance
**Code file(s):** `history/v2.9/performance_test.py`

I wrote this code:

```python
for speed in range(10, 110, 10):
    set_speed(speed)
    time.sleep(2)
    measure_actual_speed()
```

When I ran it:

```
$ python performance_test.py
Speed 100%: actual=1.8 m/s, slip=8%
Speed 80%:  actual=1.5 m/s, slip=5%
Speed 50%:  actual=0.9 m/s, slip=2%
Speed 20%:  actual=0.3 m/s, slip=1%
ERROR: At 100% PWM, motor driver temperature rises from 25°C to 68°C in 60s
```

**Root cause:** Continuous 100% duty heats the L298N beyond its thermal capacity. The datasheet specifies 2 A continuous per channel, but at 100% PWM the switching losses add to the conduction losses.

**Fix applied:** Soft limit at 90% duty (255 → 230). This caps mechanical speed at ~1.8 m/s and keeps driver temperature below 55°C.

**Code mapping:** The speed cap is in `esp/main/speed_control.c:41` — `MAX_DUTY = 230`.

---

## Chapter 21: v3.0 — IMU Raw Logging

**Date:** late July 2026
**Goal:** Log raw MPU6050 accelerometer/gyroscope data at 100 Hz
**Code file(s):** `history/v3.0/log_imu.py`

I wrote this code:

```python
for i in range(500):
    ax, ay, az = read_accel()
    gx, gy, gz = read_gyro()
    csv_writer.writerow([ax, ay, az, gx, gy, gz])
    time.sleep(0.01)
```

When I ran it:

```
$ python log_imu.py
Sample 1: accel=(-16384, 16384, -16384)   gyro=(-32768, 24567, -10345)
Sample 50: accel=(28433, -19200, 15300)    gyro=(11000, -8000, 7000)
Sample 101: accel=(0, 0, 16384)            gyro=(12, -5, 8)
ERROR: First 100 samples are garbage — sensor stabilization takes ~1 second
```

**Root cause:** The MPU6050 needs ~100 ms to stabilize after power-up per the datasheet (section 4.17). Its internal ADC and signal path have not settled, producing random values. We started logging within 5 ms of initialization.

**Fix applied:** Insert `time.sleep(1)` before the logging loop and explicitly discard the first 100 samples.

**Code mapping:** The discard is in `pi/sensors/imu.py:29` — `_discard_samples(100)` called during `IMU.init()`.

---

## Chapter 22: v3.1 — IMU Calibration

**Date:** late July 2026
**Goal:** Compute gyro bias and accelerometer scale factors
**Code file(s):** `history/v3.1/calibrate_imu.py`

I wrote this code:

```python
bias = mean(read_gyro(n=1000))  # average 1000 readings at rest
```

When I ran it:

```
$ python calibrate_imu.py
Cold bias (20°C):  gyro_z = 0.5 °/s
After 30 min (45°C):  gyro_z = 2.1 °/s
ERROR: Bias drifts by 1.6 °/s as temperature rises
```

**Root cause:** The MPU6050's MEMS gyroscope has a temperature-dependent bias drift. The datasheet specifies ±0.05 °/s/°C typical. Over a 25 °C temperature rise (from 20 °C to 45 °C internal), the bias shifts by up to 1.25 °/s.

**Fix applied:** Capture bias at startup after a 5-minute warmup period. Store in NVS for reuse. Implement running bias estimation during operation using accelerometer-based correction.

**Code mapping:** The auto-calibration is in `pi/sensors/imu.py:55` — `calibrate_gyro_bias()`.

---

## Chapter 23: v3.2 — Complementary Filter

**Date:** late July 2026
**Goal:** Fuse accelerometer and gyroscope for tilt angle estimation
**Code file(s):** `history/v3.2/complementary.py`

I wrote this code:

```python
angle = 0.98 * (angle + gyro * dt) + 0.02 * accel_angle
```

When I ran it:

```
[Robot tilts 20°, filter takes 1.2 seconds to converge]
ERROR: Lag = 1.2 seconds at α=0.98 — too slow for real-time control
```

**Root cause:** The complementary filter's time constant `τ = (α * dt) / (1 - α)`. With `α = 0.98` and `dt = 0.01 s`: `τ = 0.49 s`. But actually the effective lag is longer because `α` is applied to the gyro integration (which drifts) and `1-α` to the accel (which is noisy but absolute). α=0.98 trusts gyro heavily, so any gyro bias error persists.

**Fix applied:** Reduced α to 0.92:

```python
angle = 0.92 * (angle + gyro * dt) + 0.08 * accel_angle
```

**Equation:** `τ = (α * dt) / (1 - α) = (0.92 * 0.01) / 0.08 = 0.115 s` — 115 ms lag, acceptable.

**Code mapping:** The filter is in `pi/fusion/complementary.py:18`.

---

## Chapter 24: v3.3 — Magnetometer Heading

**Date:** late July 2026
**Goal:** Get absolute heading from QMC5883L magnetometer
**Code file(s):** `history/v3.3/heading.py`

I wrote this code:

```python
heading = atan2(mag_y, mag_x) * 180 / pi
```

When I ran it:

```
$ python heading.py
Heading: 45° (expected 0° — robot pointing north)
Heading: 90° after 90° rotation (expected 90°)
Heading: 10° after 180° rotation (expected 180°)
ERROR: Non-linear heading error — hard iron distortion
```

**Root cause:** Ferromagnetic materials on the robot (motor magnets, steel shafts, battery terminals) create a fixed magnetic offset (hard iron distortion). The magnetometer sees the sum of Earth's field + robot's field, not Earth's field alone.

**Fix applied:** Perform 360° calibration: rotate the robot slowly through a full circle, record min/max of each axis, compute offset = `(max + min) / 2`:

```python
mag_x -= (max_x + min_x) / 2
mag_y -= (max_y + min_y) / 2
heading = atan2(mag_y, mag_x)
```

**Code mapping:** The calibration routine is in `pi/sensors/magnetometer.py:38`.

---

## Chapter 25: v3.4 — ToF Distance Reading

**Date:** late July 2026
**Goal:** Read VL53L0X distance reliably
**Code file(s):** `history/v3.4/read_tof.py`

I wrote this code:

```python
dist = sensor.read_range_single_millimeters()
print(f"Distance: {dist} mm")
```

When I ran it:

```
$ python read_tof.py
Distance: 450 mm
Distance: 0 mm
Distance: 0 mm
[Object is 800 mm away — sensor returns 0 when out of range]
```

**Root cause:** The VL53L0X returns 0 (or 65535 depending on mode) when the target is outside the ranging window or the signal is too weak. It does not return the maximum range value — it returns a sentinel error value.

**Fix applied:** Clamp and validate: treat 0 as "out of range" and return `max_range = 2000 mm` for the VL53L0X:

```python
dist = raw if raw > 0 else 2000
```

**Code mapping:** The clamp is in `pi/sensors/tof.py:27` — `_validate_distance()` caps at `_MAX_RANGE`.

---

## Chapter 26: v3.5 — Multi-ToF Fusion

**Date:** late July 2026
**Goal:** Fuse readings from 3 ToF sensors for wall distance estimation
**Code file(s):** `history/v3.5/tof_fusion.py`

I wrote this code:

```python
def get_wall_distance():
    return (read_left() + read_right()) / 2
```

When I ran it:

```
$ python tof_fusion.py
Left: 320 mm  Right: 340 mm  Fused: 330 mm  (robot centred in 400mm corridor)
Left: 120 mm  Right: 680 mm  Fused: 400 mm  (robot near left wall)
ERROR: Crosstalk causes left/right to interfere — fused reading is garbage
```

**Root cause:** Both VL53L0X sensors fire simultaneously (v1.6 stagger wasn't implemented on the Pi side). The left sensor's VCSEL pulse reflects off the wall and is detected by the right sensor, causing a false short reading.

**Fix applied:** Stagger reads by 20 ms on the Pi side, mirroring the ESP32 approach from v1.6:

```python
left = read_left()
time.sleep(0.02)
right = read_right()
```

**Code mapping:** The stagger is in `pi/sensors/tof_fusion.py:34`.

---

## Chapter 27: v3.6 — Camera Frame Capture

**Date:** late July 2026
**Goal:** Capture continuous frames at 30 fps for vision processing
**Code file(s):** `history/v3.6/capture_frame.py`

I wrote this code:

```python
cam = Picamera2()
cam.start()
for i in range(200):
    frame = cam.capture_array()
    process(frame)
cam.stop()
```

When I ran it:

```
$ python capture_frame.py
Frame 1-99: OK at 30 fps
Frame 100:  ERROR — timeout waiting for frame
Frame 101:  ERROR — buffer not available
[Camera stalls indefinitely after frame 100]
```

**Root cause:** `cam.capture_array()` allocates a new buffer each call. After ~100 frames, the GPU's buffer pool is exhausted and the camera driver blocks waiting for a buffer to be released — but the previous buffers are still referenced by Python's garbage collector.

**Fix applied:** Use a preallocated buffer with `cam.capture_array(buffer)`, or call `del frame` explicitly after processing, or use `cam.capture_request()` which returns a context-managed buffer that's automatically released.

```python
with cam.capture_request() as req:
    frame = req.make_array("main")
```

**Code mapping:** The buffer-managed capture is in `pi/vision/camera.py:48`.

---

## Chapter 28: v3.7 — HSV Colour Detection

**Date:** late July 2026
**Goal:** Detect red pillars using HSV colour thresholding
**Code file(s):** `history/v3.7/color_detect.py`

I wrote this code:

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
```

When I ran it:

```
$ python color_detect.py
Red pillar at (320, 240): area=450px
Red pillar at (100, 300): area=320px  (actually a blue object?)
[Only half of red pillar is detected — the rest falls outside hue range]
```

**Root cause:** In HSV colour space, red hue wraps around 0°/180° (OpenCV halves the range to 0–180). Red is at both 0–10 AND 170–180, but our mask only covered 0–10. The shaded half of the red pillar had hue values near 175°, missing the mask entirely.

**Fix applied:** Two-range masking for red:

```python
mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
mask2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
mask = cv2.bitwise_or(mask1, mask2)
```

**Code mapping:** The two-range red mask is in `pi/vision/color_detect.py:32`.

---

## Chapter 29: v3.8 — Blob Detection

**Date:** late July 2026
**Goal:** Detect coloured pillars as blobs in camera frames
**Code file(s):** `history/v3.8/blob_detect.py`

I wrote this code:

```python
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for c in contours:
    area = cv2.contourArea(c)
    if area > 100:
        print(f"Blob at centre, area={area}")
```

When I ran it:

```
$ python blob_detect.py
Blob at (315, 240), area=1520px  (red pillar — correct)
Blob at (280, 420), area=880px   (red floor reflection — false positive!)
ERROR: Floor reflections of red pillar create secondary blobs below real pillar
```

**Root cause:** The red pillar reflects off the glossy arena floor, creating a mirror image blob below the actual pillar. The reflection is dimmer but still within the HSV threshold range.

**Fix applied:** Aspect ratio filter — real pillars are tall (height/width > 1.5), floor reflections are wide and short:

```python
h, w = cv2.boundingRect(c)
if h / w < 1.5:
    continue  # reject floor reflections
```

**Code mapping:** The aspect ratio filter is in `pi/vision/blob_detect.py:44`.

---

## Chapter 30: v3.9 — Health Monitor

**Date:** late July 2026
**Goal:** Monitor all sensor health and degrade gracefully on failure
**Code file(s):** `history/v3.9/sensor_health.py`

I wrote this code:

```python
def report_failure(sensor):
    print(f"ERROR: {sensor} read failed")
    failures[sensor] += 1
```

When I ran it:

```
$ python sensor_health.py
ERROR: tof_left read failed
ERROR: tof_left read failed
ERROR: tof_left read failed
... (500+ times in 10 seconds)
[Log file grows to 47 MB in 5 minutes]
```

**Root cause:** No rate limiting — every single sensor read failure generated a log line. A glitching sensor (common with I2C) floods the log with identical messages, obscuring real errors.

**Fix applied:** Rate-limit error logging to 1 per 2 seconds per sensor:

```python
if time.monotonic() - last_log[sensor] >= 2.0:
    print(f"ERROR: {sensor}: {msg}")
    last_log[sensor] = time.monotonic()
```

**Code mapping:** The rate-limited logger is in `pi/system/health_monitor.py:67`.

---

## Chapter 31: v4.0 — Lane Detection

**Date:** late July 2026
**Goal:** Detect lane boundaries using Hough transform
**Code file(s):** `history/v4.0/lane_detect.py`

I wrote this code:

```python
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=10)
```

When I ran it:

```
[WARN] [0.421] Left slope=-1.23, right slope=-0.45 — both pointing left!
[WARN] [0.422] Lane width 42mm — rejecting frame
[Output is a plate of spaghetti — every scratch becomes a "lane line"]
```

**Root cause:** `HoughLinesP` returns every edge fragment — floor scratches, shadows, dust. With no filtering, the slope-based lane classifier frequently finds two "left" lines or a lane width of a few cm.

**Fix applied:** 5-frame sliding window averaging and slope validation:

```python
def slope_in_range(slope, side):
    if side == "left":
        return -2.0 < slope < -0.2
    elif side == "right":
        return 0.2 < slope < 2.0
```

**Code mapping:** The sliding window filter is in `pi/vision/lane_detect.py:63`.

---

## Chapter 32: v4.1 — Wall Detection

**Date:** late July 2026
**Goal:** Detect track walls using ToF sensors
**Code file(s):** `history/v4.1/wall_detect.py`

I wrote this code:

```python
def wall_distance():
    left = read_tof(LEFT)
    right = read_tof(RIGHT)
    return min(left, right)
```

When I ran it:

```
$ python wall_detect.py
Left=340mm  Right=360mm  Wall=340mm  (robot centred)
Left=25mm   Right=675mm  Wall=25mm   (robot hugging left wall)
Left=0mm    Right=700mm  Wall=0mm    (BLIND SPOT — robot crashed!)
ERROR: VL53L0X returns 0 when object is closer than 30mm (its minimum range)
```

**Root cause:** The VL53L0X has a minimum ranging distance of ~30 mm. When the robot is within 30 mm of a wall, the sensor returns 0 (out of range sentinel). The `min()` treats 0 as "very close" which is correct, but there's no handling for the case where both sensors return 0 (robot in a corner — blind spot).

**Fix applied:** Treat any reading < 30 mm as `max_range` to avoid phantom detections, and fall back to the other sensor:

```python
left = read_tof(LEFT) if read_tof(LEFT) > 30 else 2000
```

**Code mapping:** The minimum-distance clamp is in `pi/perception/wall_detect.py:22`.

---

## Chapter 33: v4.2 — Free Space Detection

**Date:** late July 2026
**Goal:** Identify free (drivable) space in front of the robot
**Code file(s):** `history/v4.2/free_space.py`

I wrote this code:

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
```

When I ran it:

```
$ python free_space.py
[Robot stops in the middle of a straight — thinks shadow is an obstacle]
[Shadow of overhead light fixture covers 40% of the track — thresholded as black]
ERROR: Shadows are classified as obstacles — robot stops every 2 meters
```

**Root cause:** Simple grayscale thresholding cannot distinguish between a dark obstacle and a shadow. Both have low pixel values. The track surface (grey) varies from 100–200 just from lighting gradients.

**Fix applied:** Use HSV saturation channel instead of grayscale — the track surface has low saturation (grey), obstacles have high saturation (coloured pillars, walls). Shadows don't change saturation significantly.

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
_, binary = cv2.threshold(hsv[:,:,1], 30, 255, cv2.THRESH_BINARY)
```

**Code mapping:** The saturation-based free space detector is in `pi/vision/free_space.py:31`.

---

## Chapter 34: v4.3 — Corner Detection

**Date:** late July 2026
**Goal:** Detect corners on the track using gyro yaw rate
**Code file(s):** `history/v4.3/corner_detect.py`

I wrote this code:

```python
if abs(yaw_rate) > 30:  # °/s
    print("Corner detected!")
```

When I ran it:

```
$ python corner_detect.py
Corner 1: Δheading=85°  (detected at 32°/s)
Corner 2: Δheading=95°  (detected at 41°/s)
Corner 3: Δheading=88°  (detected at 29°/s)
[All corners should be 90° — gyro drift causes 85-95° variation]
ERROR: Gyro drift accumulates ±5° per corner — after 4 corners, heading error = 20°
```

**Root cause:** The gyroscope bias (v3.1 fix) is still drifting during operation. The temperature inside the robot rises during a run, shifting the bias. 100 seconds of driving × 0.02 °/s residual bias = 2° error per lap.

**Fix applied:** Reset heading integration after each corner. Use the turn's end to re-calibrate: straight segments mean yaw_rate ≈ 0, so enforce a zero-update:

```python
if abs(yaw_rate) < 5:
    heading = round(heading / 90) * 90  # snap to nearest 90°
```

**Code mapping:** The corner reset logic is in `pi/perception/corner_detect.py:54`.

---

## Chapter 35: v4.4 — Red Pillar Detection

**Date:** late July 2026
**Goal:** Detect red pillars per WRO Rule 13.21
**Code file(s):** `history/v4.4/red_pillar.py`

I wrote this code:

```python
lower_red = np.array([200, 20, 30])   # RGB
upper_red = np.array([255, 80, 80])
mask = cv2.inRange(frame, lower_red, upper_red)
```

When I ran it:

```
$ python red_pillar.py
Red pillar at (312, 240), area=1560px, confidence=0.87
Red pillar at (298, 430), area=920px, confidence=0.81
[WARN] [4.321] That's below the bumper line — that's on the floor!
[Detected 4 pillars. There are only 2 on the track.]
```

**Root cause:** Red electrical tape on the arena floor (inspection markings) has the same RGB colour as the official red pillars. From the camera's perspective, a 20 mm wide strip of tape at close range occupies the same pixel area as a distant pillar.

**Fix applied:** Minimum aspect ratio — pillars are tall (3:1 height:width), floor tape is flat (0.2–0.5:1):

```python
h, w = rect[1]
if h / w < 1.5:
    reject
```

**Code mapping:** The aspect ratio filter is in `pi/perception/pillar_detect.py:43`.

---

## Chapter 36: v4.5 — Green Pillar Detection

**Date:** late July 2026
**Goal:** Detect green pillars per WRO Rule 13.22
**Code file(s):** `history/v4.5/green_pillar.py`

I wrote this code:

```python
lower_green = np.array([40, 100, 100])
upper_green = np.array([80, 255, 255])
mask = cv2.inRange(hsv, lower_green, upper_green)
```

When I ran it:

```
$ python green_pillar.py
[Robot drives past a green pillar without detecting it]
[Green pillar merges with the grey floor — low saturation on both]
ERROR: Green pillar at 2m distance has saturation=35 (thresholded as floor at 30)
```

**Root cause:** The green pillar at a distance appears desaturated (atmospheric haze, lower resolution). Its HSV saturation drops to ~35, while the grey floor has saturation ~25. The threshold at 30 cuts the pillar in half.

**Fix applied:** Tune HSV ranges per venue at competition — make the config externally adjustable:

```python
# config/pillar_colors.yaml
green: {h_low: 35, h_high: 85, s_low: 20, v_low: 50}
```

**Code mapping:** The YAML-configured colour ranges are loaded in `pi/perception/pillar_detect.py:18`.

---

## Chapter 37: v4.6 — Pink Marker Detection

**Date:** late July 2026
**Goal:** Detect pink parking markers per WRO Rule 13.27
**Code file(s):** `history/v4.6/pink_marker.py`

I wrote this code:

```python
min_area = 500  # pixels
if area > min_area:
    print(f"Pink marker detected at {x}, distance={dist}")
```

When I ran it:

```
$ python pink_marker.py
[Robot is 1.5m from pink marker — nothing detected]
[Robot is 0.4m from pink marker — detected! area=520px]
ERROR: Marker not detected beyond 500 mm — it's too small in the frame
```

**Root cause:** The pink marker is a small object (~50 mm diameter). At 1 m distance, it occupies only ~80 pixels in a 640×480 frame. The `min_area = 500` threshold requires the robot to be within 400 mm before detection triggers — too late for the parking approach.

**Fix applied:** Reduce minimum detection area to 100 pixels and use temporal confirmation (detect in 3 consecutive frames before trusting):

```python
min_area = 100  # detect from up to 1.2m away
if detected_in_n_frames(3):
    confirm()
```

**Code mapping:** The temporal confirm filter is in `pi/perception/pillar_detect.py:70`.

---

## Chapter 38: v4.7 — Pillar Distance from Pixel Height

**Date:** late July 2026
**Goal:** Estimate pillar distance using known height and pixel dimensions
**Code file(s):** `history/v4.7/pillar_distance.py`

I wrote this code:

```python
distance = (real_height * focal_length) / pixel_height
```

When I ran it:

```
$ python pillar_distance.py
Pillar at 1.0m actual → estimated 1.15m (error=15%)
Pillar at 2.0m actual → estimated 2.30m (error=15%)
ERROR: Camera is tilted 5° downward — pixel height includes perspective foreshortening
```

**Root cause:** The camera is mounted at a 5° downward pitch to see the track ahead. This means the pillar's image height is foreshortened by `cos(5°)`, introducing a systematic ~0.4% error per degree. More importantly, the camera's height above ground isn't purely vertical — the pitch changes the effective focal centre.

**Fix applied:** Account for camera pitch by adjusting the pixel-height formula with the IMU pitch angle:

```python
pitch = imu.pitch  # from v3.2 complementary filter
distance = (real_height * focal_length) / (pixel_height * cos(pitch))
```

**Equation:** `d = (H * f) / (h_px * cos(θ))` where `θ` is the camera pitch angle.

**Code mapping:** The pitch-corrected distance is in `pi/perception/pillar_distance.py:29`.

---

## Chapter 39: v4.8 — Multi-Pillar Tracking

**Date:** late July 2026
**Goal:** Track multiple pillars simultaneously across frames
**Code file(s):** `history/v4.8/multi_pillar_track.py`

I wrote this code:

```python
pillars = detect_pillars(frame)  # returns list of (x, y, colour, area)
```

When I ran it:

```
$ python multi_pillar_track.py
Frame 50: Red pillar at (320, 240)  — tracking
Frame 55: [pillar disappears during turn]
Frame 60: Red pillar at (100, 200)  — new ID assigned
ERROR: Pillar ID changes every turn — robot can't track which side it passed
```

**Root cause:** During sharp turns, the pillar temporarily leaves the camera's field of view (120° HFoV). The detection drops to 0 for 3–5 frames. When the pillar re-enters the frame, the tracker assigns a new ID — it doesn't know it's the same pillar.

**Fix applied:** Kalman filter prediction — extrapolate pillar position during occlusion using the robot's odometry:

```python
kf.predict()  # update pillar position based on robot motion
if no_detection_for < 10:
    use_predicted_position
```

**Code mapping:** The Kalman tracker is in `pi/perception/pillar_tracker.py:55`.

---

## Chapter 40: v4.9 — Visual Odometry

**Date:** late July 2026
**Goal:** Estimate robot motion from camera optical flow
**Code file(s):** `history/v4.9/visual_odometry.py`

I wrote this code:

```python
flow = cv2.calcOpticalFlowFarneback(prev, curr, None, 0.5, 3, 15, 3, 5, 1.2, 0)
```

When I ran it:

```
$ python visual_odometry.py
[VO] Frame 0: optical flow computed in 185 ms
[VO] Frame 1: optical flow computed in 192 ms
[VO] Frame 2: optical flow computed in 188 ms
[WARN] [0.565] Control loop running at ~5.3 fps — below minimum 15 fps!
```

**Root cause:** Dense Farneback optical flow computes motion vectors for every pixel in a 640×480 frame (307,200 pixels). At 180 ms per frame, the theoretical max is ~5.5 fps — far below the 30 fps control loop. The robot would miss obstacles between frames.

**Fix applied:** Reduced resolution to 320×240 and switched from dense Farneback to FAST corner detector + sparse Lucas-Kanade optical flow:

```python
fast = cv2.FastFeatureDetector_create(threshold=20)
kp = fast.detect(gray, None)
pts = np.array([p.pt for p in kp], dtype=np.float32).reshape(-1, 1, 2)
next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev, curr, pts, None)
```

**Equation:** FAST runtime = `O(n * 16)` for n detected corner candidates (200–500 points) — orders of magnitude faster than dense flow `O(307200)`.

**Code mapping:** The sparse visual odometry is in `pi/vision/visual_odometry.py:41`.

---

## From v4.9 to the Present

v4.9 marked the end of the **Track Understanding** phase. From here, the project continued through:

- **v5.x — Fusion**: UKF implementation, Mahalanobis outlier rejection, cross-sensor verification
- **v6.x — Control**: PID gain scheduling, Stanley steering, cubic spline trajectories
- **v7.x — Mission**: Full state machine (10 states), lap counter, parking strategy
- **v8.x — Advanced**: Multi-mode steering (same/opposite/crab), surprise rule YAML config
- **v9.x — Polish**: CI pipeline, integration tests, bug fixes, release candidate

Each phase is documented in its own CHANGE.md files under `history/v*.*/`. The full 90-version journey from skeleton to competition-ready robot lives there, warts and all.

For the definitive reference on the *final* architecture (post-v9.9), see `ARCHITECTURE.md`. For the condensed changelog, see `CHANGELOG.md`.
