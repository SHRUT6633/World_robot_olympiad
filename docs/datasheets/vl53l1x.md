<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/datasheets/vl53l1x.md
Component: VL53L1X ToF sensor (front)
=============================================================================
-->

# VL53L1X Time-of-Flight Sensor (Front)

Long-range front detection for obstacle avoidance and the black track
entry/exit decision.

## Key electrical specs

| Parameter | Value |
|-----------|-------|
| Supply voltage | 2.6–3.5V (we run 3.3V) |
| Average current | ~25 mA |
| Peak current (active ranging) | ~30 mA |
| Interface | I2C (400 kHz) |
| Range | up to 4000 mm with reflector |
| Update rate | up to 50 Hz |
| Light source | 940 nm VCSEL, laser class 1 |
| Logic level | 3.3V |

## Power rail connection

- Pi GPIO 3.3V pin (sensor rail).

## Interface

- SDA = Pi GPIO 2, SCL = Pi GPIO 3.
- Address selection via XSHUT = BCM23.

## Protection notes

- Highest current of the four sensor types — the sensor rail total
  stays under 60 mA, within the Pi 3.3V pin budget.

## Official datasheet

- https://www.st.com/resource/en/datasheet/vl53l1x.pdf
