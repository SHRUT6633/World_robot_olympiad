# v1.7 — GPIO LED + Switch Test

## Testing Basic Digital I/O

Before we could implement the robot's user interface, we needed to verify that the Raspberry Pi's GPIO pins can reliably read switch inputs and control LEDs. The robot has two status LEDs (green for "ready/ok", red for "error/fault") and one start switch (momentary push button) that the driver presses to begin the race.

Our test was simple: blink the green LED 5 times (500ms on, 500ms off), then wait for the start switch to be pressed. When the switch is pressed, the LED turns solid on and the script exits. This test exercises both output (LED control) and input (switch reading) with pull-up resistors.

## First Error: Switch Bouncing

When we first ran the test, pressing the switch once caused multiple detections. The script printed "Switch detected!" multiple times for a single press, and the LED toggle would cycle rapidly as if the switch was being pressed repeatedly. The problem was mechanical switch bounce.

A mechanical push button consists of two metal contacts that touch when pressed. When the contacts come together, they do not make a clean electrical connection. Instead, they bounce (separate and reconnect) for several milliseconds before settling. During this bouncing period, the GPIO pin sees a rapid sequence of HIGH and LOW transitions. Without debouncing, the software interprets each transition as a separate press event.

Using an oscilloscope, we measured the bounce duration at approximately 15-20ms for our particular switch model. The bounces were most severe in the first 5ms after the initial contact, then tapered off. Some bounces caused the signal to go completely HIGH (open circuit) for up to 2ms before reconnecting.

## The Fix: Software Debounce

We implemented a software debounce function that reads the pin multiple times at short intervals and requires a majority of reads to agree before reporting the value:

```python
def debounce_read(pin, samples=5, interval=0.01):
    results = [GPIO.input(pin) for _ in range(samples)]
    return 1 if sum(results) > samples // 2 else 0
```

This function reads the pin 5 times at 10ms intervals (total 50ms), then returns 1 (HIGH) if at least 3 of the 5 reads were HIGH, or 0 (LOW) otherwise. The 10ms interval is longer than the worst-case bounce duration (20ms), so by the time we take the third sample, the switch should have settled. The majority voting ensures that a single noise spike does not corrupt the reading.

The debounce adds 50ms latency to switch detection, meaning the robot's software sees the switch press 50ms after it actually happened. For a start switch, this latency is irrelevant — the robot starts a race at most once, and 50ms is negligible compared to the race duration (several minutes). For an emergency stop switch, 50ms is still acceptable because the robot travels only 2.5cm at 0.5 m/s during that time.

## Alternative: Hardware RC Debounce

We considered using a hardware debounce circuit: a resistor-capacitor (RC) low-pass filter connected to the switch output, followed by a Schmitt trigger to clean up the signal. The RC filter (e.g., 10kΩ resistor + 1μF capacitor, giving a time constant of 10ms) would smooth out the bounces, and the Schmitt trigger (e.g., 74HC14) would produce a clean digital edge.

Hardware debounce has the advantage of zero software overhead and no latency beyond the RC time constant. However, it requires additional components (resistor, capacitor, Schmitt trigger IC) and PCB space. For a single switch, the software debounce approach is simpler and more flexible — we can adjust the debounce parameters by changing code rather than soldering new components.

## Pin Configuration

The GPIO pins were configured as follows:

- Green LED (GPIO 23): Output, active-high. Connected via 220Ω current-limiting resistor to LED anode. LED cathode to ground.
- Red LED (GPIO 24): Output, active-high. Same configuration as green LED.
- Start Switch (GPIO 25): Input with internal pull-up resistor enabled. Switch connects the pin to ground when pressed. This "active-low" configuration means the pin reads HIGH when the switch is not pressed and LOW when pressed.

The internal pull-up resistor on the Raspberry Pi is approximately 50kΩ, which is sufficient for a switch. We verified that the pull-up brings the pin to 3.3V (HIGH) when the switch is open, and the pin drops to 0V (LOW) when the switch is closed. The 50kΩ pull-up draws only 66μA when the switch is closed, which is negligible for power consumption.

## Wiring and Noise

We initially ran unshielded jumper wires from the Pi's GPIO header to the LEDs and switch, which were mounted on a prototype board about 15cm away. With the motors running (v1.3), we observed false switch readings: the GPIO pin would briefly go LOW even when the switch was not pressed. The motor's electromagnetic interference was coupling into the switch wire.

We fixed this by using twisted-pair wires for the switch connection (signal + ground in the same twisted pair) and adding a 100nF ceramic capacitor between the switch pin and ground at the Pi side. The capacitor filters out high-frequency noise from the motors. The 100nF capacitor and the 50kΩ pull-up resistor form a low-pass filter with a cutoff frequency of approximately 32Hz, which is well below the motor PWM frequency of 1000Hz.

## Learned: Always Debounce Switches

The main lesson from v1.7 is that mechanical switches always bounce, and software debounce is essential for reliable operation. The debounce parameters (number of samples, interval) depend on the switch's physical characteristics, which we measured with an oscilloscope. For our switch, 5 samples at 10ms intervals worked reliably, but a different switch might require different parameters.

We also learned that GPIO readings can be affected by electrical noise from motors and other high-current components. Proper wiring (twisted pairs, bypass capacitors) and shielded cables are important for reliable digital I/O in a robotics environment. The RC filter on the switch input not only debounces the mechanical contact but also filters out EMI from the motors.

## Power Consumption of LEDs

The green and red LEDs are standard 5mm through-hole LEDs with a forward voltage of approximately 2.0V (green) and 1.8V (red). With the 220Ω series resistors and 3.3V GPIO output, the current through each LED is approximately (3.3V - 2.0V) / 220Ω = 5.9mA (green) and (3.3V - 1.8V) / 220Ω = 6.8mA (red). This is well within the Raspberry Pi's GPIO maximum current rating of 16mA per pin and 50mA total across all pins. The total current for both LEDs is approximately 12.7mA, which is acceptable. We verified that the LED brightness is sufficient for visibility in a well-lit room (500 lux). In direct sunlight (outdoor competition venues), the LEDs may be difficult to see. We are considering replacing the red LED with a brighter 10mm LED or adding a transparent light pipe to improve visibility.

## Alternative to RPi.GPIO: gpiozero and lgpio

The RPi.GPIO library we used in this test is popular but has known limitations: it does not support hardware-timed PWM on all pins, and it requires root privileges to access the GPIO registers. For the production code, we plan to migrate to the lgpio library, which provides better performance, support for all 40 pins, and does not require root if the user is in the gpio group. The gpiozero library (built on top of lgpio) provides a higher-level interface with built-in debounce for buttons and PWM for LEDs. However, for the initial hardware test, RPi.GPIO was sufficient and more widely documented. We will migrate to lgpio in v2.x when we implement the PWM-based LED brightness control (dimming the green LED to indicate battery level).

## Future Use in the Self-Test

The LED and switch test forms the first stage of the self-test sequence (v1.8). At boot, the green LED blinks to indicate that the software is starting, then the robot waits for the start switch. If the red LED ever lights up, it indicates a hardware fault that must be resolved before the race. The debounce function is reused throughout the codebase wherever switch inputs are read.
