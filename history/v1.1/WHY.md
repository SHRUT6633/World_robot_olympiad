# Version 1.1 — The L298N Fight

## What Happened

We plugged the motor driver. We wrote the code. The motor moved forward.
We were happy. Then we tried reverse. Nothing.

For 4 hours.

## The L298N Driver

The L298N is a cheap motor driver. It has IN1, IN2, ENA pins.
- IN1 high, IN2 low → forward
- IN1 low, IN2 high → reverse
- ENA must be HIGH with PWM for speed control

We connected ENA to 5V (always on). But the L298N needs a PWM signal
on ENA to enable the H-bridge for both directions. Without PWM, the
bridge is "off" for one direction.

Fix: Connect ENA to a GPIO pin and send PWM. We used LEDC peripheral
on ESP32 (channel 0, 50 Hz).

## Why I Chose L298N Over Other Drivers

**Option A: L298N.** Cheap ($3). Handles 2A continuous. Works at 5-12V.
Easy to find in any electronics shop. But inefficient (drops voltage).

**Option B: BTS7960.** More expensive ($8). Handles 43A (way more than
we need). Needs separate 5V supply for logic. Overkill.

**Option C: L9110S.** Cheap ($2). Good for small motors. But our motor
draws ~1A under load, which is at the limit.

I chose L298N because it is well documented and we had one lying around.
If I were doing it again, I might use L9110S for size or BTS7960 for
reliability. But L298N works if you know how to wire it (lesson: we
did not know how to wire it).

## Self-Test Addition

In v1.0 we just started driving. No check if the servo is alive, if
the motor spins, if the ESP32 is even running. That is risky.

In v1.1 I added `selftest.c`. When you press the start button:
1. Servo sweeps full range (0° → 60° → 0°)
2. Motor spins forward 1 second, reverse 1 second
3. Green LED blinks 3 times

If any step fails, red LED stays on and the robot refuses to start.

This saved us at least twice during testing when a servo wire came loose.

## UART Reliability

The Pi sends commands like "speed=0.5, steering=10". But sometimes the
ESP32 would read weird values. Like steering=99999. That would make the
robot turn full lock and crash.

**Root cause:** Electrical noise on the UART line. A start bit gets
misread, shifting everything.

**Fix:** Added CRC-16 checksum to every packet. Before the ESP32
executes a command, it calculates the checksum and compares. If it
mismatches, the packet is dropped and the last valid command continues.
Also added a 500ms timeout: if no valid packet arrives in 500ms, the
robot stops.

## What I Learned

1. Always read the datasheet. The L298N datasheet shows exactly how ENA
   should be wired. I did not read it. I wasted 4 hours.
2. Self-tests are not optional. They are like checking your parachute
   before jumping.
3. UART is not reliable by itself. Add checksums. Always.
4. Keep a backup of everything. We corrupted our SD card twice during
   testing.

## Change Summary

| File | What Changed | Why |
|------|-------------|-----|
| esp/main/l298n.c | Added PWM on ENA pin | Motor would not reverse |
| esp/main/selftest.c | NEW | Check hardware before race |
| esp/main/uart_receiver.c | CRC-16 validation | Bad packets corrupt steering |
| esp/main/packet_validator.c | Range checks | Prevent invalid commands |

## Evidence of Struggle

There is a comment in l298n.c that says:
  "// NOTE: ENA must be PWM, not just HIGH. Do not remove this."

That comment exists because I removed it once, thinking it was unnecessary,
and the motor stopped reversing again. The comment is for me.
