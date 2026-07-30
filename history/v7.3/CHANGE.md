# v7.3 — Start/Finish Detection

## Diary Entry — 2026-03-17

The robot needs a way to know when the competition has started. In WRO 2026, the start signal comes from a physical button on the robot (pressed by the team member) OR from a camera-based visual signal (a colored marker shown to the robot). Today I built the start detection module that handles both.

## Hardware setup

We have two start mechanisms:

1. **Physical button** — a momentary push button connected to GPIO pin 15, pulled up internally. When pressed, the pin goes low.
2. **Camera marker** — the camera detects a green ARUCO marker placed in front of the robot. When the marker is visible for 1 second continuously, the start is triggered.

Both signals feed into `StartDetector`, which debounces and validates them before signaling the state machine to move from IDLE to START_SEARCH.

## The switch bounce nightmare

I tested the physical button first. Simple polling loop: read GPIO, if low for 3 consecutive samples (10ms each), trigger start. Easy, right?

The log told a different story:

```
[INFO] Start signal detected! t=0.000s
[INFO] Start signal detected! t=0.035s
[INFO] Start signal detected! t=0.072s
```

Three starts within 72ms. The button was pressed once. What happened?

Switch bounce. When a mechanical switch closes, the contacts don't make clean contact — they "bounce" (rapidly open and close) for several milliseconds before settling. A single press produces dozens of transitions.

I used a simple 10ms polling loop, and the bounce was causing the GPIO to oscillate between high and low for about 15-20ms after each press. Each low reading in the oscillating period was enough to satisfy the "3 consecutive low samples" condition, triggering multiple start events.

I checked with an oscilloscope:

```
Channel 1 (GPIO 15): sawtooth bounce pattern, ~8 transitions over 18ms
Sampling at 100Hz: caught an average of 4-5 "low" readings per press
```

Actually, I should be more specific. The oscilloscope showed:

```
Press down:   HIGH → LOW (transition 1)
Bounce up:    LOW → HIGH (1.2ms later)
Bounce down:  HIGH → LOW (2.1ms later)
Bounce up:    LOW → HIGH (0.8ms later)
Bounce down:  HIGH → LOW (1.5ms later)
... (continues for ~18ms total)
Settle:       LOW (stable)
```

Each bounce pair (up-down) takes 2-4ms. My 10ms sampling interval was long enough that I sometimes caught two low readings per 10ms window, sometimes one. The pattern was hard to predict, which made it even more frustrating to debug.

## The fix: two-layer debounce

I implemented a two-layer approach:

**Layer 1: Hardware debounce (RC filter).** I added a 10µF capacitor across the switch contacts with a 1kΩ resistor. This creates a low-pass RC filter with a time constant of τ = RC = 10ms. The capacitor charges/discharges slowly, smoothing out the bounce transients. The Schmitt trigger input on the GPIO then snaps cleanly between HIGH and LOW without oscillation.

**Layer 2: Software debounce (50ms settling window).** Even with the hardware fix, I wanted software insurance. The software debounce requires the signal to be stable for 50ms (5 consecutive samples at 10ms intervals) before triggering:

```python
DEBOUNCE_MS = 50
SAMPLE_INTERVAL_MS = 10

def process_gpio_sample(self, raw_level):
    now = time.monotonic()
    elapsed = (now - self.last_sample_time) * 1000
    if elapsed < SAMPLE_INTERVAL_MS:
        return False

    if raw_level == self.last_raw_level:
        self.stable_count += 1
    else:
        self.stable_count = 0
        self.last_raw_level = raw_level

    self.last_sample_time = now

    if self.stable_count >= DEBOUNCE_MS / SAMPLE_INTERVAL_MS:
        if raw_level == self.trigger_level:
            return self._fire_start()
        self.stable_count = 0

    return False
```

## The camera start edge case

The camera marker detection had its own set of issues. The ARUCO marker detection runs at 15 FPS. If the marker is only partially visible (e.g., the robot is at an angle), the detector might see it for 2 frames, lose it for 1 frame, then see it again. This pattern could also trigger multiple starts.

I added a "confidence window" for the camera: the marker must be continuously visible for 15 consecutive frames (1 second at 15 FPS) before triggering. Any gap resets the counter.

## Alternatives considered

**Alternative 1: Capacitive touch sensor.** A capacitive touch sensor mounted on the robot's chassis. No mechanical parts, no bounce. But capacitive sensors are sensitive to moisture and nearby metal, and the competition field might have both.

**Alternative 2: IR break-beam.** An IR LED + phototransistor pair that the team member breaks with their hand. Simple, fast, but adds component cost and wiring complexity.

**Alternative 3: Bluetooth start signal.** Start via a Bluetooth command from the team's phone/laptop. Too many failure modes: pairing issues, interference, battery on the phone.

**Alternative 4: Two-layer debounce (chosen).** Hardware RC filter + software debounce. The hardware handles the high-frequency bounce, the software catches any remaining glitches. Both are well-understood techniques.

## Testing

I tested 500 button presses with a mechanical switch jig. The two-layer debounce eliminated all false triggers. Without the fix, 34% of presses caused double-triggers. With the fix: 0%.

For the camera, I recorded a 30-second video of the ARUCO marker being shown to the camera with interruptions (waving hand, partial occlusion). The confidence window correctly rejected all false positives and still triggered within 1.5s of the marker being steadily shown.

## Stats

- Lines of code: 132 (start_detect.py)
- Hardware debounce: RC filter (10µF + 1kΩ, τ=10ms)
- Software debounce window: 50ms
- Camera confidence: 15 consecutive frames
- False trigger rate: 0% (was 34%)

The robot now reliably starts on command. Tomorrow: obstacle strategy — which way do we go around things?

— 2026-03-17, signing off.
