# Test Tools

This directory contains hardware testing and calibration scripts.

## Tools

- `i2c_scan.py` — Scan I2C bus for connected sensors
- `camera_test.py` — Capture test frame from PiCamera
- `uart_test.py` — Test UART loopback with ESP32
- `sensor_read_all.py` — Read all I2C sensors in a loop
- `led_test.py` — Test GPIO LEDs and switch

## Usage

Run from the `pi/` directory:
```
python tools/i2c_scan.py
```
