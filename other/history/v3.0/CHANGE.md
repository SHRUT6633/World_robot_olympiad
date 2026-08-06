# v3.0 — IMU Raw Data Logging

## What Changed
We introduced the first sensor integration onto our WRO 2026 robot platform. The MPU6050 accelerometer and gyroscope module was wired to the Raspberry Pi 4's I2C bus (pins 3 SDA, 5 SCL), and we wrote `log_imu.py` to poll both sensors at 100 Hz and dump everything into a timestamped CSV file for offline analysis. This is the foundation of the SENSING THE WORLD phase—without reliable raw data, nothing else works.

The script uses the `smbus2` library to communicate over I2C, reads the MPU6050's ACCEL_XOUT_H through GYRO_ZOUT_L registers, converts raw 16-bit two's-complement values to physical units (±2g for accelerometer, ±250°/s for gyro), and appends each sample as a row in the CSV. We also log a millisecond-resolution timestamp so we can later verify the 100 Hz sampling rate is actually being met.

## Why
Before v3.0, our robot had no awareness of its own motion. The previous phase (SENSING THE WORLD hadn't started yet) was purely about motor control—we could drive straight-ish with open-loop PWM, but the robot couldn't tell if it was tipping over, turning too fast, or stuck against a wall. The IMU is the cheapest way to get basic orientation and angular velocity data. We need the raw log to understand noise characteristics, bias offsets, and sampling jitter before we can build any filter.

## Errors Encountered

### Garbage First Readings
The very first bug appeared immediately. Every run produced a CSV where rows 1-100 or so had accelerometer values like `-16384, 16384, -16384` (which is exactly the raw sensitivity at ±2g, 16384 LSB/g), then suddenly settled to reasonable values near `0, 0, 16384` (gravity pointing down, Z ≈ 1g). The gyro showed a similar pattern: first 100 rows showed wild swings like `-32768, 24567, -10345`, then stabilized near zero.

```
ERROR: Raw accel Z at sample 1: 32767 (impossible value)
ERROR: Raw accel Z at sample 50: 28433 (still garbage)
ERROR: Raw accel Z at sample 101: 16384 (normal)
```

I initially thought the I2C clock speed was too high (400 kHz instead of 100 kHz). I changed `smbus2.SMBus(1)` and tried setting the clock via `ioctl` to 100 kHz. No improvement. I then suspected a wiring issue—maybe the AD0 pin was floating and the address was toggling. Adding a pull-down resistor to AD0 didn't help either.

After reading the MPU6050 datasheet more carefully, I found the answer in section 4.17: "The MPU6050 requires a stabilization period of approximately 100 ms after power-up before sensor registers contain valid data." The sensor's internal ADC needs time to settle. We were starting logging immediately after initializing the I2C bus, sometimes within 5 ms.

**Fix:** Insert a delay of 1 second before the logging loop, and discard the first 100 samples explicitly. This is a hack but it works reliably. The real fix would be to check the MPU6050's signal path reset bit (register 0x68 bit 7), but the simple discard approach is good enough for now.

```python
time.sleep(1)  # let sensor stabilize
for i in range(100):
    read_mpu6050()  # discard
```

### I2C Read Failures Intermittently
After about 30 seconds of logging, we started seeing occasional `OSError: [Errno 121] Remote I/O error`. This turned out to be the Raspberry Pi's I2C clock stretching timeout. The MPU6050 sometimes holds the clock line low while processing a reading, and the Pi's default timeout is too short at 25 ms.

**Fix:** Increased the I2C timeout to 1 ms per byte in `/boot/config.txt` with `dtparam=i2c_arm=on,i2c_arm_baudrate=100000`. Then set the `i2c_arm_baudrate` to 50000 (50 kHz) as a workaround.

### 100 Hz Not Achievable
With pure Python `smbus2` calls, the effective sampling rate was about 65 Hz. Each `read_i2c_block_data` takes ~5 ms, times 6 registers (accel X/Y/Z, gyro X/Y/Z), plus CSV write overhead. We're not meeting 100 Hz.

**Fix for now:** Log to a preallocated NumPy array in RAM, then flush to CSV after the run. Also, batch-read all 6 axes in a single I2C transaction (the MPU6050 supports burst reads starting at register 0x3B). This brought us to ~95 Hz.

## Alternatives Considered
- Using the MPU6050's built-in FIFO (hardware buffer that can store 32 samples and interrupt the Pi when full). This would let us read in bulk at 1 kHz then down-sample. I decided against it because we need timestamps per sample, and the FIFO only records sample count, not absolute time.
- Switching to a BMI088 (used in many robomaster boards) for higher temperature stability. The BMI088 has separate accel and gyro dies, which reduces crosstalk. But we already had MPU6050s in stock and the budget is tight.
- Polling with a dedicated C extension or using the `pigpio` library for I2C bit-banging at higher speed. That adds build complexity. We'll revisit if 95 Hz isn't enough for the complementary filter.
- Using the Adafruit CircuitPython MPU6050 library instead of raw smbus2. It handles the register math automatically and includes a calibration helper. We chose raw smbus2 to understand every byte on the wire.

## Current Status
`log_imu.py` runs, logs ~95 Hz to CSV, discards first 100 samples, handles I2C errors with a 3-retry backoff. We have about 10 MB of CSV data per 5-minute run. The next step is to analyze this data offline to determine bias and noise characteristics, then write the calibration routine.
