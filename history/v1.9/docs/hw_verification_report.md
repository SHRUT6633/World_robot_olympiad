# Hardware Verification Report

## Tested Components (14/14 PASS)

| Component | Interface | Address/Pin | Status | Notes |
|-----------|-----------|-------------|--------|-------|
| Raspberry Pi 4 | - | - | PASS | Boots in 12s |
| ESP32-S3 | UART | /dev/serial0 | PASS | Pong response 5ms |
| PiCamera v3 | CSI | - | PASS | 640x480@60fps |
| MPU6050 IMU | I2C | 0x68 | PASS | Accel+gyro, 100Hz |
| QMC5883L Mag | I2C | 0x0D | PASS | Heading ±2° |
| VL53L0X Left | I2C | 0x30 | PASS | Range 30-2000mm |
| VL53L0X Right | I2C | 0x31 | PASS | Range 30-2000mm |
| VL53L1X Front | I2C | 0x32 | PASS | Range 40-4000mm |
| L298N Motor | GPIO | IN1=4, IN2=5, ENA=6 | PASS | Fwd+Rev, 0-100% |
| Servo | GPIO | PWM=7 | PASS | ±30°, 50Hz |
| Green LED | GPIO | 23 | PASS | On/Off |
| Red LED | GPIO | 24 | PASS | On/Off |
| Start Switch | GPIO | 25 | PASS | Debounced 50ms |
| Power 5V | - | - | PASS | 3.3A peak draw |

## Known Issues
1. ToF crosstalk when both VL53L0X fire simultaneously (stagger 20ms)
2. IMU gyro bias drifts with temperature (recalibrate at venue)
3. Camera needs 500 lux minimum (bring track lighting)
4. Motor draws 2A stall current (fuse rated 5A, OK)

## Decision
Hardware is stable enough for software development. Proceed to v2.x.
