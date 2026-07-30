<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/research/engineering_journey.md
Rev:  v9.9  |  Status: RELEASED
=============================================================================
-->

# Engineering Journey — From First Sketch to Final Architecture

## Preface

This document captures the research phase that happened **before version 1.0** of the code. It is the story of how I went from a blank sheet of paper to a complete robot architecture — without cutting a single piece of metal or writing a single line of firmware. Every design decision in this document was made on paper, in spreadsheets, and in Python simulation. By the time I ordered the first component, I knew exactly what the robot would be.

---

## Chapter 1: The First Sketch (June 2026)

It started with a pencil drawing on graph paper. A rectangle with four circles — two at the front, two at the back. A small box on top for the Raspberry Pi. A bigger box underneath for the battery. That was it. I had no idea if it would work.

The WRO 2026 rules were clear: autonomous, four wheels, single propulsion motor (Rule 11.3). Everything else was up to me.

**My first thought:** Build a simple car. Two front wheels that steer. Two rear wheels that drive. Like every robot I had seen at previous competitions. Simple. Proven. Safe.

I spent the first week collecting reference designs from previous WRO competitors. Almost all of them used 2WS. Almost all of them struggled with the obstacle challenge.

**Red flag number one.**

---

## Chapter 2: The Turning Radius Problem (June 2026, Week 2)

I drew the track layout from the rulebook on a large sheet of paper. Scaled 1:1. Corner radius: 500 mm. Lane width: approximately 1,000 mm. I traced a 300 mm wheelbase robot through the corner at various steering angles.

At 30° steering (already aggressive for a servo mechanism), the robot's inner wheel tracked a radius of approximately 370 mm. The corner inner wall was at 500 mm. Clearance: 130 mm. That seemed OK — until I added:

- 50 mm for sensor noise
- 30 mm for servo latency
- 20 mm for surface variation
- 40 mm for the robot's own width overhang

**Effective clearance: 130 - 50 - 30 - 20 - 40 = -10 mm.**

The robot would hit the wall. Not maybe. It would.

I tried increasing the wheelbase. 250 mm? Worse turning. 350 mm? Better turning but the robot is too long for the parking zone. 300 mm was the sweet spot — and it didn't work with 2WS.

**This was the moment 4WS became inevitable.**

---

## Chapter 3: Discovering 4WS (June 2026, Week 3)

I spent a week reading academic papers on four-wheel steering. Key findings:

- **Sano et al. (1986):** First practical 4WS system on a Honda Prelude. Reduced turning radius by 25%.
- **Furukawa et al. (1989):** Analysis of 4WS yaw-rate response. Found opposite-phase improves transient response by 40%.
- **Ackermann (1994):** Robust yaw control without rear steering angle sensor. Proved 4WS can be done with open-loop rear mapping.

**The math was transformative:**

For 2WS:   R = L / tan(δ)
For 4WS:   R = L / (tan(δ_f) + tan(δ_r))

With δ_r = -δ_f (opposite-phase):
R = L / (tan(δ) + tan(-δ))

This approaches **infinity in the denominator** — meaning the instantaneous turning centre can be placed anywhere, including at the robot's centre of mass.

**Realisation:** 4WS was not just slightly better. It was geometrically in a different league.

---

## Chapter 4: The Single-Servo Problem (June 2026, Week 4)

4WS typically requires two servos — one for front, one for rear. But I wanted a single servo for three reasons:

1. **Rule 11.5:** "No component may be added solely for the purpose of adding points." Two servos for steering could be questioned.
2. **Reliability:** One servo is half as likely to fail as two.
3. **Simplicity:** One PWM channel, one PID loop.

I spent a week designing mechanical linkages on paper. The breakthrough came when I realised: a single push-pull rod can control both front and rear if the pivot points are arranged correctly.

The geometry:
```
     Front axle
    ┌────┴────┐
    │         │
  Servo ─────┤ Push-pull rod
    │         │
    └────┬────┘
     Rear axle
```

With offset pivots, a single linear motion produces:
- Front wheels turn one direction
- Rear wheels turn the opposite direction

**Result:** 4WS with one servo. The full derivation is in `pi/dynamics/mechanical_linkage.py`.

---

## Chapter 5: The Drivetrain Puzzle (July 2026, Week 1)

With steering solved, I turned to propulsion. Three candidates:

**FWD:** Simple, but every automotive engineer knows FWD understeers. In a competition with tight corners, understeer loses time. Worse: in reverse (parking), FWD is the front wheels dragging the robot backward — unstable.

**RWD:** Better. Rear wheels push through corners. But there was a problem: weight distribution. With the battery and Pi at the front, the rear wheels would have less grip. In acceleration out of a corner, the rear could step out.

**AWD:** The ideal. All four wheels pulling. But Rule 11.3 says **one motor**. How do you drive all four wheels with one motor?

I sketched a central gearbox with shafts to each wheel. It worked on paper. But differentials are complex, heavy, and introduce backlash.

**Breakthrough:** I didn't need a differential. The WRO track has no high-speed sweeping turns. At the speeds we drive (0.3–1.8 m/s), tyre scrub in a turn is manageable. A compliant coupling (rubber joiner) between the front and rear drivetrain absorbs the 5–10% speed difference.

**Decision:** Single motor, central gearbox, shafts to all four wheels, compliant coupling. Simple. Reliable. WRO-compliant.

---

## Chapter 6: Sensor Selection (July 2026, Week 2)

The rulebook says the robot must:
- Detect red and green pillars (Rules 13.21–13.22)
- Detect magenta parking markers (Rule 13.27)
- Navigate the track autonomously
- Park within a 200 mm wide zone

**First attempt:** Camera only. A Raspberry Pi Camera Module v3 running at 640×480.

Problems:
- Close-range blind spot: camera cannot see objects closer than ~100 mm
- Lighting dependency: indoor venue lighting varies
- Frame rate limitation: 30 fps means 60 mm of travel between frames at 1.8 m/s

**Second attempt:** Add Time-of-Flight sensors. Three VL53 sensors:
- Left (VL53L0X, addr 0x30) — wall distance
- Right (VL53L0X, addr 0x31) — wall distance
- Front (VL53L1X, addr 0x32) — obstacle detection

The ToF sensors work in any lighting, measure up to 4 m, and update at 50 Hz.

**Camera for long-range** (pillar detection at 0.5–3 m)
**ToF for short-range** (wall following, parking alignment at 0–500 mm)

This sensor fusion pattern is implemented in `pi/perception/pillar_detector.py` and `pi/perception/parking_detector.py`.

---

## Chapter 7: State Estimation — EKF vs UKF (July 2026, Week 3)

The robot needs to know where it is. The WRO track has no GPS, no external markers (except pillars). The robot must localise using its own sensors.

**Dead reckoning** (encoder-based) was my first thought. Simple. Well-understood. But drift is quadratic with distance:

After 1 m: ~2 cm error
After 10 m: ~20 cm error
After 30 m (one lap): ~60 cm error

The track is 30 m. 60 cm error means the robot doesn't know which corner it's at.

**EKF** was the next thought. Fuse encoder odometry with IMU and magnetometer. Standard approach. Works well.

But: the EKF requires Jacobian matrices — derivatives of the nonlinear motion model. For a 4WS robot with three steering modes, the Jacobian changes depending on which mode is active. Computing it on a Pi 4B at 100 Hz is expensive and error-prone.

**UKF** does not need Jacobians. It propagates sigma points through the nonlinear function and reconstructs the mean and covariance. This is:
- More accurate (3rd-order vs 1st-order for EKF)
- More robust (no linearisation errors)
- Simpler to maintain (change the motion model without rewriting the filter)

**Cost:** ~2× more computation per step. But at 100 Hz, the Pi 4B handles it easily (~15% CPU).

**Decision:** UKF with 6 state dimensions (x, y, heading, speed, acceleration, yaw rate). Implemented in `pi/fusion/ukf.py`.

---

## Chapter 8: The Parking Challenge (July 2026, Week 3)

Parking is worth 15 points. To get full marks:
- Robot must be fully inside the 200 mm wide parking zone
- Distance between left/right wheels and the wall ≤ 2 cm
- Robot must remain stationary for 30 seconds

**The 2WS parallel park problem:** A 2WS vehicle cannot move sideways. It must perform a multi-point turn (pull forward, reverse, adjust, repeat). Under autonomous control, this is unreliable — each manoeuvre accumulates error.

**The 4WS advantage:** With crab-walk mode, the robot moves sideways. Single smooth motion into the parking zone.

**The alignment problem:** How do you know you are parallel to the wall? The camera is useless at close range (less than 100 mm = blur). The solution:

1. Approach at 45° using camera to detect magenta markers (Rule 13.27)
2. At 200 mm distance, switch to ToF left/right sensors
3. Read left and right ToF simultaneously — if |left - right| ≤ 20 mm, robot is parallel
4. Drive straight into the parking zone
5. Stop when rear ToF (or odometry) indicates full insertion
6. Verify for 30 seconds

This state machine is implemented in `pi/perception/parking_detector.py` with 7 states.

---

## Chapter 9: The Surprise Rule — Designing for Adaptability (July 2026, Week 4)

WRO includes a surprise rule announced at the competition. Previous years had:
- 2023: Reverse direction of travel
- 2024: Differently coloured obstacles
- 2025: Modified parking zone dimensions

I designed the architecture so that **any** surprise rule can be handled by changing **one line** in a YAML file.

The `config/surprise_rules.yaml` file exposes 55+ parameters including:
- `pillar_logic`: NORMAL or REVERSED (swap left/right)
- `parking_approach_angle`: degrees
- `parking_tolerance_mm`: clamp distance
- `colour_thresholds`: HSV ranges for pillar detection
- `steering_mode_default`: SAME, OPPOSITE, or CRAB
- `direction_override`: FORWARD, REVERSE, AUTO

No code changes needed. One line changed. One commit. Judge can verify in 10 seconds.

---

## Chapter 10: The Final Architecture — Why It Works

After 6 weeks of research, simulation, and paper design, I had:

| Subsystem | Choice | Why Not Alternative |
|-----------|--------|---------------------|
| Steering | 4WS, single servo | 2WS turning radius insufficient |
| Drivetrain | AWD, single motor | FWD understeers, RWD oversteers |
| Sensors | Camera + 3× ToF + IMU + Mag | Redundant, lighting-independent |
| State Est. | UKF (6-DOF) | EKF needs Jacobians, less robust |
| Control | PID + Stanley + Feedforward | MPC too expensive for Pi 4B |
| Config | YAML-based | One-line surprise rule adaptation |
| Parking | Vision + ToF 7-state FSM | Camera alone fails at close range |

**The architecture is deliberately over-engineered.** Every subsystem has a backup:
- If camera fails → ToF-based wall following
- If IMU drifts → magnetometer heading
- If UKF diverges → reinitialize from encoder dead reckoning
- If servo jams → mechanical linkage defaults to straight

**This is not paranoia. This is competition engineering.** On the day, one thing will go wrong. The robot that finishes is the robot that has a plan B, C, and D.

---

## What Comes Next

This document represents the **research phase** — v0.1 through v0.9 in spirit, though the code starts at v1.0. Every insight here is reflected in the codebase:

| Research Insight | Code Implementation |
|-----------------|-------------------|
| 4WS turning radius advantage | `pi/dynamics/steering_modes.py` |
| Single-servo linkage geometry | `pi/dynamics/mechanical_linkage.py` |
| AWD with compliant coupling | `esp/main/l298n.c` |
| Sensor fusion strategy | `pi/sensors/base.py`, `pi/fusion/ukf.py` |
| Parking 7-state machine | `pi/perception/parking_detector.py` |
| Surprise rule adaptability | `config/surprise_rules.yaml` |
| Multi-rate scheduling | `pi/system/scheduler.py` |
| Failure mode handling | `pi/system/health_monitor.py` |

**The 90 versions in history/v1.0 through v9.9 show the code evolving. This document shows the mind evolving. Both are needed for the full picture.**
