# Our Robot's Story — From Zero to Something That Works

```
v1.0        v1.1        v1.2        v1.3        v1.4
 │           │           │           │           │
 │ blank     │ motor     │ import    │ typo      │ comments
 │ screen    │ driver    │ hell      │ fixed     │ + polish
 │           │           │           │           │
 ▼           ▼           ▼           ▼           ▼
[---nothing--]──→[---spins---]──→[---runs----]──→[---drives--]──→[---ready---]
    1.0          1.1          1.2          1.3          1.4
  Jul 27       Jul 28       Jul 29       Jul 29      Jul 30
                                                       ▲
                                            ──→ You are here
```

This is not a changelog. This is our diary.

Every folder in here is a snapshot of our robot at one moment in time.
If you open them one by one you will see us learn, break things, fix them,
get stuck, ask stupid questions, and slowly — very slowly — make something
that actually drives.

I wrote this so that anyone (even someone who has never written code)
can see how a robot grows. Like watching a child learn to walk.

---

## The Beginning — v1.0 (July 27)

We had a deadline. Two months until competition. We had nothing.

So we sat down and wrote the first version of everything. Not because we
knew what we were doing. Because we had to start somewhere.

**The big decision:** We chose Raspberry Pi + ESP32-S3. Why two computers?
Because the Pi is good at thinking (camera, maths) and the ESP32 is good
at moving (motors, sensors). Like having a brain and a body.

**Another big decision:** One motor for all four wheels. The rules say you
can only have one motor if they are mechanically linked. So we made a
chain drive. One motor pulls everything. This is NOT the fastest way but
it is the legal way.

**What we were scared of:** Would the Pi talk to the ESP32 fast enough?
Would the camera see the track? Would the battery die after 30 seconds?
We did not know. We just wrote code and hoped.

Code written: ~15,000 lines (most of it wrong, but that is normal).

---

## First Problems — v1.1 (July 28)

The robot started. Then it did weird things.

**Problem 1: The motor driver.**
We used an L298N module. It made the motor go forward. But when we
tried to reverse, nothing happened. We checked the wires. We checked
the code. We spent 4 hours. Finally we realized: the enable pin was
not connected to PWM. Stupid mistake. But that is how you learn.

**Problem 2: Self-test.**
We wrote a program that makes the servo sweep left-right and the motor
spin forward-backward when you press the button. This way we know
everything is alive before the race. Before this, we just hoped.

**Problem 3: UART packets.**
The Pi sends commands like "steer 15 degrees" to the ESP32. But sometimes
the ESP32 would read garbage (like "steer 1 million degrees"). We added
CRC-16 checksum. Now if a packet is corrupted, we throw it away. This is
like putting a stamp on a letter so you know if someone opened it.

**What we learned:** Hardware is not perfect. Wires come loose. Signals
get noise. You have to write code that expects things to fail.

---

## The Import Nightmare — v1.2 (July 29)

This was the most frustrating day.

We copied the code to the actual Raspberry Pi (not our laptop) and
ran it. Error. `ImportError: No module named 'sensors'`.

We spent 3 hours. We tried pip install. We tried moving files. Nothing.
Finally we realized: Python needs to know where to find our files. We
added `sys.path.insert(0, ...)` and added `pi.` in front of every import.

Why did this happen? Because on our laptop we ran the code from inside
the `pi/` folder. On the Pi we ran it from the project root. Different
starting point = different import paths. Simple when you know. But we
did not know.

**Lesson:** Always test on the real hardware, not just your computer.

---

## The Typo That Broke Everything — v1.3 (July 29)

Same day. Another error. This time: `NameError: MerkedScaledSigmaPoints`.

One letter. We typed "Merked" instead of "Merwe". It is a class name
from a library called filterpy. It does fancy maths for our Kalman
Filter. This filter is how the robot knows where it is.

Because of one typo, the robot could not localize itself. It was blind.

We fixed it in 30 seconds after we found it. But finding it took 2 hours
of reading error messages and Googling.

**Also this version:** We found all the `while True` loops. Someone (me)
wrote infinite loops inside the scheduler callbacks. This meant Ctrl+C
could not stop the robot. You had to kill the terminal. Very annoying.
We removed them all and let the scheduler control the timing.

**Lesson:** Read your code before you run it. Especially the parts you
copied from Stack Overflow at 2 AM.

---

## Making It Beautiful — v1.4 (July 29-30)

The robot worked. But the code was ugly. No comments. No explanations.

For competition, the judges look at your code. They want to see if you
understand what you wrote. So we went through every single file and added
comments. Not just "this adds 1 to x" but real explanations: WHY we add 1,
WHAT happens if we don't.

We also realized: the competition has "Surprise Rules". The judges can
change the rules on the spot. For example: "Make the robot pass pillars
on the opposite side" or "Drive clockwise instead of counter-clockwise."

We created a single config file: `config/surprise_rules.yaml`. If the
judges announce a surprise rule, we change ONE LINE and the robot adapts.
No code changes. No rushing to rewrite logic at the competition.

**The pillar detector:** We read the rulebook. It says pillars are
Red(238,39,55), Green(68,214,44), and Magenta(255,0,255). We converted
these RGB values to HSV (because OpenCV uses HSV for colour detection).
We tuned the ranges to work in different lighting.

**The parking detector:** The robot has to park between two magenta
markers. It must be parallel to the wall (less than 2 cm error) and
stay still for 30 seconds. We wrote a state machine: IDLE → MARKER_SEEN
→ BETWEEN_MARKERS → ALIGNING → BACKING_IN → PARKED → VERIFIED. Each
state checks sensors and decides what to do next.

**The track map:** We did NOT use SLAM. SLAM is a fancy algorithm that
builds a map as it goes. But it is heavy on the CPU. Instead, we know
the track dimensions from the rules. We just track how far we have gone
and which section we are in (straight, corner, obstacle zone). Simple,
fast, works.

---

## Final Thoughts

If you look at the code from v1.0 to v1.4, you will see:

1. We made mistakes. Lots of them.
2. We fixed them. Slowly.
3. We added comments because we knew judges would read them.
4. We planned for surprises because competitions are unpredictable.
5. We kept things simple because complex things break more.

The robot is not perfect. But it is ours. And it drives.

---

## How We Grew (Numbers That Matter)

| Metric | v1.0 | v1.1 | v1.2 | v1.3 | v1.4 |
|--------|------|------|------|------|------|
| Python files | 35 | 36 | 38 | 38 | 51 |
| C files | 8 | 11 | 11 | 11 | 11 |
| Comment lines | ~50 | ~50 | ~50 | ~80 | ~950 |
| Known bugs | 15+ | 8 | 5 | 2 | 0 (maybe) |
| Scheduler tasks | 6 | 6 | 6 | 7 | 9 |
| Config options | 25 | 30 | 30 | 30 | 55+ |
| Hours of sleep | 6 | 4 | 3 | 5 | 7 |
| Cups of tea | 2 | 5 | 4 | 8 | 3 |

---

## How To Read This Folder

Each subfolder (v1.0, v1.1, ...) has a complete copy of the code at that
point in time. Open the code and the `WHY.md` file together. The WHY.md
tells you what we were thinking when we wrote it.

Start from v1.0 and read forward. You will see:
- Code that is messy → code that is clean
- Features that are missing → features that work
- Confidence that is low → confidence that is ... slightly higher

---

**What I want you to take away:**

Building a robot is not about writing perfect code on the first try.
It is about writing bad code, realizing it is bad, fixing it, and writing
better code tomorrow. Every version is a step. Even the wrong steps teach
you something.

*Written July 30, 2026 — 27 days before the competition. We are not ready.
But we are readier than yesterday.*
