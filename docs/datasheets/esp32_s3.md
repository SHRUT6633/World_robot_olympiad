<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/datasheets/esp32_s3.md
Component: ESP32-S3 (custom board)
=============================================================================
-->

# ESP32-S3

The motor/servo microcontroller. Runs C firmware (ESP-IDF): L298N PWM,
servo PWM, UART packet receiver, failsafe.

## Key electrical specs

| Parameter | Value |
|-----------|-------|
| Logic voltage | 3.3V |
| Board input | 5V (VBUS via USB) |
| Typical current | 160–300 mA |
| Boot/start peak | ~500 mA (this is what burned the v1.2 board!) |
| Current with WiFi ON | 500 mA+ (WiFi is OFF during rounds) |
| GPIO level / drive | 3.3V, ~40 mA per pin |
| CPU | 2× Xtensa LX7 @ 240 MHz |

## Power rail connection

- Powered from the **Raspberry Pi USB port (5V)** — never from a
  weak 3.3V source.
- Onboard LDO makes 3.3V for the module. The ESP32 is NOT used to
  power sensors (they live on the Pi's I2C bus).

## Interface

- UART RX = GPIO17, UART TX = GPIO18 ↔ Pi BCM14/15 @ 115200.
- Servo PWM = GPIO13 (50 Hz), L298N: ENA = GPIO11, IN1 = GPIO8, IN2 = GPIO9.

## Protection notes

- If powered from a source that cannot supply ~500 mA at boot, the
  module brownout-loops and the LDO can burn (see POWER_DISTRIBUTION
  Section 8 — the v1.2 incident).

## Official datasheet

- https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
