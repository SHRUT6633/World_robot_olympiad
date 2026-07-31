<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/datasheets/drive_motor.md
Component: Drive motor (single, AWD linkage)
=============================================================================
-->

# Drive Motor (AWD)

One DC gear motor drives all four wheels through the mechanical AWD
linkage (Rules 11.4, 11.13 — one driving motor).

## Bench-measured specs

| Parameter | Value |
|-----------|-------|
| Nominal voltage | 6V |
| Free-run current | ~150 mA |
| Average load current | ~0.9 A |
| Stall current | ~2.5 A |
| Speed (no load) | ~200 RPM @ 6V |
| Reduction | ~48:1 gearbox |

## Power rail connection

- Motor rail (battery direct 7.4V) via L298N OUT1/OUT2.
- PWM speed control from ESP32 (ENA).

## Protection notes

- Motor start transient is the biggest power event on the robot —
  absorbed by the 1000µF bulk cap (v2.x brownout fix).
- Current never approaches the 10A fuse, but the stall case is exactly
  why the fuse exists.

## Official datasheet

- None — generic TT gear motor. Values above are bench measurements
  from this robot, taken with a current clamp.
