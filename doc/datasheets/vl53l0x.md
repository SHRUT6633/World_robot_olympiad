<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/datasheets/vl53l0x.md
Component: VL53L0X ToF sensor (left + right)
=============================================================================
-->

# VL53L0X Time-of-Flight Sensor

Left and right wall/pillar detection for lane keeping and obstacle
avoidance.

## Key electrical specs

| Parameter | Value |
|-----------|-------|
| Supply voltage | 2.6–3.5V (we run 3.3V) |
| Average current | ~6 mA |
| Peak current (measurement) | ~20 mA |
| Interface | I2C (400 kHz) |
| Range | 30–2000 mm (up to ~3000 mm, long-range mode) |
| Update rate | up to 50 Hz |
| Light source | 940 nm VCSEL, laser class 1 |
| Logic level | 3.3V |

## Power rail connection

- Pi GPIO 3.3V pin (sensor rail) — two units.

## Interface

- SDA = Pi GPIO 2, SCL = Pi GPIO 3.
- Unique I2C addresses via XSHUT pins: Left = BCM17, Right = BCM27.

## Protection notes

- Each measurement bursts ~20 mA — covered by the sensor rail budget
  (< 60 mA total with IMU + mag + front ToF).

## Official datasheet

- https://www.st.com/resource/en/datasheet/vl53l0x.pdf
