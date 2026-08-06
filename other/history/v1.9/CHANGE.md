# v1.9 — Hardware Verification Complete

## Summary

v1.9 marks the completion of the Foundation & Hardware Testing phase of the WRO 2026 robot project. Over the course of 10 iterations (v1.0 through v1.9), we have:

- Designed the dual-board architecture (Raspberry Pi 4 + ESP32-S3)
- Established the project directory structure and development workflow
- Tested all 14 hardware components individually and as an integrated system
- Identified and resolved 8 critical hardware/software issues
- Created a comprehensive self-test that runs at every startup
- Documented known limitations and risk factors

This milestone represents the transition from hardware validation to software development. All hardware components are verified to work, the communication protocol between Pi and ESP32 is stable, and the self-test ensures that hardware failures are caught before the robot enters the competition track.

## 14 Hardware Components Verified

The hardware verification report documents each component's test results. All 14 components passed their individual tests and the integrated self-test. The components are:

1. Raspberry Pi 4 (main computer, runs Python, OpenCV, control logic)
2. ESP32-S3 (microcontroller, runs FreeRTOS, handles real-time I/O)
3. PiCamera v3 (CSI camera, 640x480@60fps, BGR888 format)
4. MPU6050 IMU (I2C 0x68, accelerometer and gyroscope, 100Hz)
5. QMC5883L Magnetometer (I2C 0x0D, compass heading, ±2° accuracy)
6. VL53L0X Left (I2C 0x30, ToF sensor, 30-2000mm range)
7. VL53L0X Right (I2C 0x31, ToF sensor, 30-2000mm range)
8. VL53L1X Front (I2C 0x32, ToF sensor, 40-4000mm range)
9. L298N Motor Driver (GPIO IN1=4, IN2=5, ENA=6, PWM 1000Hz)
10. Steering Servo (GPIO 7, 50Hz PWM, ±30° range)
11. Green LED (GPIO 23, status indicator)
12. Red LED (GPIO 24, fault indicator)
13. Start Switch (GPIO 25, debounced input)
14. 5V Power Supply (3.3A peak draw, separate motor/logic rails)

## Known Issues

### 1. ToF Crosstalk Between VL53L0X Sensors

When both VL53L0X sensors fire simultaneously (distance measurement laser pulses overlap in time), they can interfere with each other. The VL53L0X uses a VCSEL (Vertical-Cavity Surface-Emitting Laser) that emits 940nm infrared light. If both sensors emit laser pulses at the same time, each sensor may detect the other's pulse as a reflection, resulting in false short-distance readings.

The fix is to stagger the firing of the two sensors by 20ms. This is implemented in the ESP32's sensor polling loop: it reads the left sensor first, waits 20ms, then reads the right sensor. The 20ms delay increases the total cycle time from 10ms to 30ms for the ToF sensors, but the fast sensors (MPU6050, QMC5883L) continue to read at 100Hz.

At the competition venue, we will test for crosstalk by placing an obstacle at a known distance and verifying that both sensors report the correct distance simultaneously. If crosstalk persists (e.g., due to reflective surfaces in the environment), we can further increase the stagger to 30ms.

### 2. IMU Gyro Bias Drift

The MPU6050's gyroscope exhibits bias drift with temperature. When the robot is cold (at room temperature, 20°C), the gyro bias is approximately ±0.5 °/s. After 30 minutes of operation, the internal temperature rises to about 45°C, and the bias drifts to ±2 °/s. This bias drift affects the orientation estimate used for stability control.

We compensate for gyro bias by calibrating at startup: the robot must remain stationary for 2 seconds while the gyro readings are averaged. This average bias is subtracted from all subsequent gyro readings. However, as the temperature changes during operation, the bias shifts and the calibration becomes less accurate.

For the competition, we will run the robot for 5 minutes before the race to let the temperature stabilize, then recalibrate the gyro. This is documented in the competition checklist. If we have time before the competition, we may implement a running bias estimator that slowly updates the bias estimate during operation (using accelerometer data to correct for gyro drift).

### 3. Camera Needs 500 Lux Minimum

The PiCamera v3's image quality degrades significantly below 500 lux. In low light, the automatic gain control increases the analog gain, which amplifies sensor read noise. The noise appears as random pixel variations that can cause false line detections in the vision pipeline.

The competition venue's lighting is unknown. We will bring two portable LED light panels (each providing approximately 1000 lux at 1m distance) to ensure the track is adequately lit. The lights will be mounted on the robot, angled downward at 45 degrees to illuminate the track surface without creating glare.

If the venue has natural lighting (windows), the color temperature may differ from our laboratory's fluorescent lighting. The camera's auto white balance should compensate, but we will perform a white balance calibration at the venue before the race.

### 4. Motor Stall Current

The L298N motor driver is rated for 2A continuous per channel. Our motors draw approximately 1.2A at normal load and 2.1A under stall conditions. If the robot gets stuck (e.g., wedged against an obstacle), the motor current could exceed the L298N's rating, triggering thermal shutdown (typically after 5 seconds of stall).

We have a 5A fuse in the motor power line to protect against catastrophic short circuits, but the fuse does not trip at 2.1A (it would take hours). The software must detect a stall condition (motor commanded to move but no encoder feedback or no position change detected by the ToF sensors) and stop the motor within 1 second. This stall detection will be implemented in the software phase (v2.x).

## Risk Assessment

### Competition Risks

1. **Camera failure**: If the camera fails during a race, the robot cannot detect the line. Probability: low (camera is solid-state, no moving parts). Mitigation: spare camera on hand, self-test before each race.

2. **UART disconnection**: The 4-wire cable (power, ground, TX, RX) between Pi and ESP32 could be pulled loose during transport. Probability: moderate (cable is exposed). Mitigation: secure cable with zip ties, check connection before race, use locking JST connectors.

3. **Battery voltage drop**: The 11.1V LiPo battery discharges during use, and the 5V buck converter may drop out below 7V input. Probability: low if battery is fully charged. Mitigation: monitor battery voltage via ADC, display warning on LED (red blink) when below 7.5V.

4. **Sensor interference**: Other robots at the competition may use similar I2C sensors with the same addresses. Probability: low (other robots use different hardware). Mitigation: our sensors are on I2C addresses uncommon for consumer modules.

### Development Risks

1. **Integration complexity**: The dual-board architecture introduces more failure modes than a single-board solution. Each board-to-board interaction (UART, I2C, power) is a potential point of failure.

2. **Timing dependencies**: The control loop depends on sensor data arriving within specific time windows. If the UART link becomes congested (e.g., during debug logging), sensor updates may be delayed.

3. **Library compatibility**: Future updates to Raspberry Pi OS, Picamera2, or smbus2 may introduce breaking changes. We have pinned library versions in requirements.txt.

## Decision: Proceed to Software Development

The hardware is stable enough for software development. All components pass the self-test, the communication protocol works reliably, and the known issues have workarounds. We now move to the Software Development phase (v2.x), which will focus on:

- Line detection and following algorithms (OpenCV, color thresholding)
- Sensor fusion (combining IMU, magnetometer, and ToF data)
- PID control loops for steering and speed
- Obstacle detection and avoidance
- State machine for race phases (start, line follow, obstacle, stop)
- Competition-specific logic (WRO 2026 rules)

The self-test from v1.8 will run at every boot, ensuring that hardware failures are caught immediately. The calibration procedures from v1.4 (servo) and v1.6 (sensor timing) will be refined during software testing. The hardware verification report (v1.9) serves as the definitive reference for all hardware components and their known limitations.
