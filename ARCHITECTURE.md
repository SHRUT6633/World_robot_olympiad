<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: ARCHITECTURE.md
Rev:  v9.9  |  Status: RELEASED
=============================================================================
-->
# System Architecture

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 4                              │
│                                                                     │
│  ┌──────────┐   ┌──────────────┐   ┌────────────┐   ┌───────────┐ │
│  │ CAMERA   │   │  ToF (I2C)   │   │ IMU (I2C)  │   │ MAG (I2C) │ │
│  │ 640×480  │   │ L/R/Front    │   │ MPU6050    │   │ QMC5883L  │ │
│  │ @60fps   │   │ VL53L0X/L1X  │   │ acc+gyro   │   │ heading   │ │
│  └────┬─────┘   └──────┬───────┘   └─────┬──────┘   └─────┬─────┘ │
│       │                │                 │                │        │
│       ▼                ▼                 ▼                ▼        │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    SENSOR BUFFER + SYNC                     │   │
│  │         (FrameSync, SensorBuffer, TimestampSync)            │   │
│  └────────────────────────┬────────────────────────────────────┘   │
│                           │                                        │
│          ┌────────────────┼────────────────┐                       │
│          ▼                ▼                ▼                       │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────┐           │
│  │  FUSION      │ │ PERCEPTION   │ │  LOCALIZATION    │           │
│  │  ────────    │ │ ──────────   │ │  ────────────    │           │
│  │  • UKF (6DoF)│ │ • PillarDet  │ │  • PoseEstimator │           │
│  │  • Complement│ │ • PillarTrk  │ │  • TrackMap      │           │
│  │  • AdaptNoise│ │ • ParkingDet │ │  • FeatureMap    │           │
│  │  • Mahalanob │ │ • LaneDet    │ │  • OccupancyGrid │           │
│  │              │ │ • WallDet    │ │  • LoopClosure   │           │
│  │  Output:     │ │ • FreeSpace  │ │                  │           │
│  │  filtered    │ │              │ │  Output:         │           │
│  │  pose        │ │ Output:      │ │  section name,   │           │
│  │  (x,y,head)  │ │ detections   │ │  lap count, pos  │           │
│  └──────┬───────┘ └──────┬───────┘ └────────┬─────────┘           │
│         │                │                   │                     │
│         ▼                ▼                   ▼                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                     MISSION (StateMachine)                  │   │
│  │  INIT→IDLE→FORWARD→CORNERING→OBSTACLE_AVOID→PARK→SHUTDOWN │   │
│  └────────────────────────┬────────────────────────────────────┘   │
│                           │                                        │
│                           ▼                                        │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              PLANNING + TRAJECTORY                          │   │
│  │  GlobalPlanner → WaypointGenerator → CubicSplineTrajectory  │   │
│  │                      → VelocityProfiler                     │   │
│  └────────────────────────┬────────────────────────────────────┘   │
│                           │                                        │
│                           ▼                                        │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              DYNAMICS + CONTROL                              │   │
│  │  SteeringMode → KinematicModel → StanleyController          │   │
│  │                                    → ServoPID + MotorPID    │   │
│  └────────────────────────┬────────────────────────────────────┘   │
│                           │                                        │
│                           ▼                                        │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   UART COMMUNICATION                        │   │
│  │              Packet framing, CRC-16, 115200 baud             │   │
│  └────────────────────────┬────────────────────────────────────┘   │
└───────────────────────────┼─────────────────────────────────────────┘
                            │ UART TX/RX
                            ▼
┌────────────────────────────────────────────────────────────────────┐
│                        ESP32-S3                                    │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │                 UART RECEIVER + PACKET VALIDATOR             │ │
│  │                    CRC check → command dispatch              │ │
│  └────────┬──────────────────────────────────┬──────────────────┘ │
│           │                                  │                     │
│           ▼                                  ▼                     │
│  ┌──────────────────┐              ┌──────────────────┐           │
│  │   SERVO PWM      │              │   L298N MOTOR    │           │
│  │   50 Hz, 1-2ms   │              │   PWM + DIR      │           │
│  │   one servo      │              │   one motor      │           │
│  │   (Rule 11.3)    │              │   (Rule 11.3)    │           │
│  └────────┬─────────┘              └────────┬─────────┘           │
│           │                                  │                     │
│           ▼                                  ▼                     │
│     ┌─────────┐                       ┌──────────┐                │
│     │ Servo   │                       │ DC Motor │                │
│     │ (4WS)   │                       │ (AWD)    │                │
│     └─────────┘                       └──────────┘                │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │             SAFETY LAYER                                     │ │
│  │  Watchdog (3s) + TimeoutDetector (500ms) + Failsafe          │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

## Subsystem Dependency Graph

```
sensors/  ──→ fusion/  ──→ localization/  ──→ mission/  ──→ planning/
   │                       ▲                                        │
   │                       │                                        │
   └──────────────────→ perception/                                  │
                                                                    ▼
                                                            dynamics/
                                                               │
                                                               ▼
                                                           control/
                                                              │
                                                              ▼
                                                          comm/uart/
                                                              │
                                                              ▼
                                                          ESP32-S3
```

## Threading Model

```
Main Thread (async)
├── BootSequence (one-shot)
├── scheduler.task("sensors",     hz=100)   — Read all I2C + camera
├── scheduler.task("fusion",      hz=100)   — UKF + complementary filter
├── scheduler.task("perception",  hz=50)    — Lane/pillar/parking detection
├── scheduler.task("planning",    hz=50)    — Waypoint + trajectory
├── scheduler.task("control",     hz=100)   — Stanley + PID output
├── scheduler.task("comms",       hz=50)    — UART TX/RX
├── scheduler.task("health",      hz=10)    — Heartbeat + diagnostics
└── scheduler.task("logging",     hz=1)     — Log state summary
```

## Key Design Decisions

1. **No SLAM** — Track map uses geometry-based section tracking (faster, lighter, deterministic)
2. **Single-loop callbacks** — No `while True` in callbacks; scheduler owns timing via `hz`
3. **Rate-limited error logging** — Max 1 error / 2s per sensor; auto-disable after 50 consecutive failures
4. **4WS via steering modes** — Three modes (SAME_PHASE, OPPOSITE_PHASE, CRAB_WALK) selectable from config
5. **Surprise Rule config** — Change one line in `config/surprise_rules.yaml` to adapt to any announced rule
6. **Exact rulebook colours** — Pillar RGB values from Rulebook 13.21-13.22 converted to HSV via calibration
7. **Parking verification** — Must be stationary for 30 s + parallel alignment ≤ 2 cm for full 15 pts
