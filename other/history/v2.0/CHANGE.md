# v2.0 — Simple Forward Drive

## What changed

Today was the day I finally got the robot to move under its own power. After weeks of skeleton code, wiring diagrams, and "we'll fix it in software" promises, I wrote the first real driving code. The concept was laughably simple: the Raspberry Pi sends a speed command over UART, the ESP32 receives it and drives the motor at a fixed PWM duty cycle. Forward. That's it. No steering, no sensors, no feedback. Just wheels spinning.

I started on the Pi side. The `drive_forward.py` script opens the UART port (`/dev/ttyAMA0` on the Pi 5, baud rate 115200) and sends a JSON message. The protocol we settled on in v1.0 uses newline-delimited JSON. The message for v2.0 is: `{"cmd": "drive", "speed": 50}`. Speed is a percentage, 0-100. The script sends this in a loop every 100 ms for 5 seconds, then sends `{"cmd": "drive", "speed": 0}` to stop.

On the ESP32 side, I wrote `motor_driver.c`. It listens on UART for incoming commands, parses the JSON using a minimal JSON parser and sets the PWM duty cycle on the motor pin. The motor driver is an L298N H-bridge. I configured the ESP32's MCPWM peripheral to generate a 50 Hz PWM signal on GPIO 25 (the enable pin for channel A). The two direction pins (GPIO 26 and 27) are set high and low respectively for forward motion.

The PWM resolution is 8 bits, so the duty cycle ranges from 0 to 255. A speed of 50% maps to duty 127. The MCPWM timer runs at 50 Hz, which is the standard frequency for servo control. We're using the same timer for the steering servo later, so 50 Hz it is.

## Why it changed

This was the logical next step after v1.0. We had a skeleton. Now we needed movement. Every robot needs to drive before it can do anything else. The line-following camera pipeline can't be tested until the robot can actually move along a line. The PID controller can't tune itself on a stationary robot. Everything depends on having reliable basic motion.

## Errors encountered

Oh, the brownout. The beautiful, infuriating brownout.

The first test was on my desk. The robot was propped up on a cardboard box so the wheels were in the air. I ran the script. The wheels spun. I cheered. Then I put the robot on the floor and ran it again.

It moved about 10 centimeters and died.

The ESP32's USB serial console printed:

```
Brownout detector was triggered
Brownout detector was triggered
```

And then the device disappeared from `/dev/ttyACM0`. The ESP32 had reset itself. This happened because the motor was drawing too much current at full PWM (100% duty), which caused the 3.3V regulator on the ESP32 dev board to drop out of regulation. The ESP32's brownout detector kicked in and reset the chip.

I measured the current draw with a multimeter. At 100% PWM, the L298N was drawing about 1.8A from the 5V rail. The ESP32-S3 dev board is powered by a separate 3.3V regulator that gets its input from the same 5V rail. When the motor kicked in, the 5V rail dipped to about 4.2V, which is below the dropout voltage of the 3.3V regulator (AMS1117-3.3, dropout voltage ~1.1V). So the 3.3V rail dropped below 3.0V and the ESP32 had a nervous breakdown.

The fix was simple in concept but took three tries to get right. I added a speed ramp-up: instead of jumping straight to the target PWM, the ESP32 ramps up over 500 milliseconds. The ramp is linear: start at duty 0, increase by `target_duty / 50` every 10 ms (50 steps over 500 ms). This limits the inrush current because the motor has time to start spinning and generate back-EMF, which reduces the effective voltage across the winding resistance.

```c
for (int i = 0; i < 50; i++) {
    int duty = (target_duty * i) / 50;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

The first attempt used 50 ms steps (25 steps over 1250 ms), which worked but was too slow—the robot took over a second to reach full speed. The second attempt used 5 ms steps, but the `vTaskDelay(5)` was unreliable because FreeRTOS tick rate is 100 Hz (10 ms per tick). The delay rounded up to 10 ms anyway. The third attempt settled on 10 ms steps, which gives exactly the 500 ms ramp.

## Alternative approaches considered

I considered three alternatives to the software ramp:
1. **Hardware solution**: Add a large capacitor (e.g., 4700 µF) across the 5V rail to absorb the current spike. This would work but takes up physical space and delays the inevitable—the battery voltage still sags under sustained load.
2. **Separate power supply**: Use a dedicated battery for the motors and another for the logic. We already discussed this in the design phase and decided against it because of weight and complexity. The robot has to stay under 1 kg for the competition.
3. **Lower PWM frequency**: Running the motor at a lower PWM frequency (e.g., 20 Hz) would reduce switching losses in the H-bridge and maybe keep the voltage higher. But 20 Hz is audible and annoying, and the motor would run less efficiently.

I went with the software ramp because it costs nothing (no extra hardware), is easily tunable (change the ramp time in code), and is a common technique in robotics.

## Reasoning

The ramp-up fix is a pragmatic solution to a power problem. The ESP32 brownout detector is there for a reason—it protects the chip from running at an undervoltage condition where flash reads become unreliable. But the real problem is that our power budget assumed the motor would draw 500 mA continuous, and the stall current is 2.0 A. We need to account for transient loads in our power budget.

I also added a startup delay in `drive_forward.py`: the Pi now waits 2 seconds after opening the UART port before sending the first command. This gives the ESP32 time to boot up and initialize the MCPWM peripheral. Without this delay, the first command would arrive while the ESP32 was still in its bootloader, and the JSON parser would choke on garbage output from the boot ROM.

The 500 ms ramp is conservative. I timed it with a stopwatch and the robot reaches full speed at about the 500 ms mark, smoothly. No more brownouts.
