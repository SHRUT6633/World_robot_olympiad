# v2.9 — Drive Reliability

## What changed

This is a consolidation release. No new features. Instead, I ran a full battery of reliability tests and documented everything. The basic driving phase is complete. The robot can drive forward, turn, stop, reverse, and accelerate smoothly under manual or programmed control. All known bugs from v2.0 through v2.8 are fixed.

I wrote `drive_reliability.md` as a comprehensive summary of the driving subsystem: architecture, performance numbers, known issues, and test results. This document is the handoff to the next phase (Basic Line Following), where the vision team will build the camera pipeline that drives the robot based on line position.

## Why it changed

Before moving to the next development phase, I needed to:
1. **Verify all fixes are stable**: The brownout fix from v2.0, the Ackermann fix from v2.1, the PWM configuration from v2.2, the encoder fix from v2.3, the PID windup fix from v2.4, the timing fix from v2.5, the braking fix from v2.6, the S-curve from v2.7, and the keyboard fix from v2.8—all needed to work together without regression.
2. **Benchmark performance**: What's the maximum speed? What's the minimum turning radius? How accurate is the odometry? These numbers are needed for the path planner.
3. **Document known issues**: The PWM whine, the L298N thermal limits, the IMU drift—these aren't fixed but are documented so the team knows the constraints.
4. **Create a stable baseline**: Future development starts from this point. If we introduce a bug in v3.x, we can revert to v2.9 and know the basics work.

## Errors encountered

The regression testing revealed three issues that I had to fix:

**Regression 1: UART buffer overflow.** In v2.8, I combined drive and steer into one message. But the ESP32's UART driver only has a 256-byte receive buffer. At 50 Hz, each message is about 40 bytes, so 2000 bytes/second. The buffer fills up in 128 ms. If the ESP32's processing loop takes longer than that (which it does when parsing commands, polling IMU, reading encoders, and updating PWM), the buffer overflows and messages are lost.

The fix was to increase the UART buffer size from 256 to 1024 bytes:
```c
uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
```
This gives 512 ms of buffer at the current data rate, which covers the worst-case processing delay (about 300 ms when IMU and encoder reads coincide).

**Regression 2: PID integral windup still possible with gyro drift.** The heading computed by the gyro integration drifts about 1 degree per minute. Over a 5-minute run, that's 5 degrees of heading error. The PID sees this as a persistent error and adjusts the motor speeds to turn the robot. But the robot was actually driving straight—the gyro was wrong. The integral term would accumulate this phantom error and cause the robot to slowly veer.

The fix: I added a high-pass filter to the gyro data to remove the DC offset (bias). The IMU is calibrated at startup: the gyro is read for 2 seconds while stationary, and the average reading is subtracted from all future readings. This reduces the drift to about 0.1 degree per minute.

**Regression 3: S-curve + dynamic brake interaction.** When the robot decelerates with an S-curve and then brakes at the end, there was a brief moment (about 10 ms) where the S-curve was still reducing speed and the brake kicked in simultaneously. This caused a jerk spike because the brake's reverse polarity was fighting the forward motor drive.

The fix: when braking is requested, the S-curve accelerates the deceleration to zero speed instantaneously (within one control loop cycle), and then the brake is applied. This prevents the two controllers from fighting.

## Performance numbers

The final benchmark results (documented in detail in `drive_reliability.md`):

**Maximum speed**: 1.8 m/s at 100% PWM, measured over a 10 m straight run with odometry and confirmed with a stopwatch.

**Minimum turning radius**: 
- SAME_PHASE steering (both front wheels turn in the same direction for parallel parking): 0.8 m radius.
- OPPOSITE_PHASE steering (one wheel forward, one backward for zero-radius turning): 0.5 m radius. This uses differential drive by reversing one drive motor. It's hard on the motors but works for tight maneuvers.

**Odometry accuracy**: ±2% over 10 m straight run, ±5% over a path with turns.

**PID heading hold**: ±1.2 degrees at 0.5 m/s, ±3.5 degrees at 1.0 m/s.

**Stop distance (with brake)**: 2 cm at 10% speed, 12 cm at 100% speed.

**Brownout threshold**: No brownouts observed in 50 consecutive start-stop cycles.

**UART reliability**: 0 lost messages in 10 minutes of continuous 50 Hz operation (after buffer fix).

## Alternative approaches considered

This version didn't introduce new approaches—it's a consolidation. But I did consider whether to refactor the ESP32 firmware into a state machine (vs. the current linear command-response model). The state machine would be more robust for handling complex command sequences. I decided against it because the current model works and changing it now would risk breaking everything. The state machine can wait for v3.x if needed.

## Reasoning

The drive reliability tests confirm that the basic driving subsystem is ready for the next phase. The maximum speed of 1.8 m/s is comfortably above the 1.5 m/s that the competition course demands. The minimum turning radius of 0.5 m (OPPOSITE_PHASE) can handle the sharpest turns on the WRO course (which are about 0.6 m radius).

The known issues are all documented:
1. **PWM whine at 50 Hz**: The motors are audible. This is cosmetic.
2. **L298N thermal limit**: More than 5 aggressive stops in quick succession will trigger thermal shutdown. The 1-second cooldown mitigates this.
3. **IMU heading drift**: 0.1 degree per minute after calibration. Acceptable for course runs under 2 minutes.
4. **Odometry slip error**: 2-5% error from wheel slip. The camera will correct this in the next phase.
5. **Servo nonlinearity**: The servo doesn't perfectly follow the commanded angle (about 2 degrees error at extreme angles). The calibration lookup table compensates for most of this.

The code at this point is clean enough that I'm comfortable handing it off to the vision team. The `driver/` module has a clear API: `drive(speed, steering_angle)`, `stop(brake=True)`, `get_odometry()`, `get_heading()`. Everything else is internal. The vision team doesn't need to know about Ackermann geometry, PID tuning, or UART protocols. They just call `driver.drive(speed, steer)` and the robot goes.

v2.9 marks the end of the Basic Driving phase. Next up: Basic Line Following.
