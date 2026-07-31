<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/datasheets/raspberry_pi_4b.md
Component: Raspberry Pi 4 Model B
=============================================================================
-->

# Raspberry Pi 4 Model B

The main computer. Runs all Python code: sensors, fusion, perception,
localization, control, mission. Also the **power hub** for all logic
(see Section 8 of POWER_DISTRIBUTION.md).

## Key electrical specs

| Parameter | Value |
|-----------|-------|
| Supply voltage | 5V (USB-C, official PSU 5V / 3A) |
| Idle current | ~600 mA |
| Headless + camera current | ~1.3 A avg, 2.0 A peak |
| GPIO logic level | 3.3V |
| 3.3V pin total budget | ~50 mA (shared by all sensors!) |
| USB port output | 600 mA default, up to 1.2 A with PD negotiation |
| SoC | BCM2711, 4× Cortex-A72 @ 1.8 GHz, 4 GB RAM |

## Power rail connection

- 5V rail from buck converter → GPIO `5V` pin (pin 2 or 4).
- ESP32-S3 powered from one of the Pi's USB ports.
- All I2C sensors powered from GPIO `3.3V` pin (pin 1).

## Interface

- I2C1: SDA = GPIO 2 (pin 3), SCL = GPIO 3 (pin 5), 400 kHz.
- UART0 `/dev/serial0`: TX = GPIO 14, RX = GPIO 15, 115200 baud → ESP32.

## Protection notes

- Brownout below ~4.6V under load → SD card corruption. The 5V rail
  must never sag (bulk caps, Section 4 of POWER_DISTRIBUTION.md).

## Official datasheet

- Product brief: https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf
- Docs: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
