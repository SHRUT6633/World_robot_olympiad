# v1.1 — I2C Bus Scanner

## Purpose

The goal of v1.1 was to detect and verify all I2C sensors on the robot's bus. Our hardware design includes five I2C devices: an MPU6050 IMU for acceleration and gyroscope data, a QMC5883L magnetometer for compass heading, two VL53L0X Time-of-Flight sensors for left and right obstacle detection, and one VL53L1X Time-of-Flight sensor for front-facing long-range detection. All five devices share the same I2C bus (bus 1 on the Raspberry Pi 4, mapped to GPIO 2 (SDA) and GPIO 3 (SCL)).

Before we could write high-level sensor fusion code, we needed to confirm that each sensor responds at its expected address. The MPU6050 should be at 0x68 (AD0 pin pulled low), the QMC5883L at 0x0D (default address), the two VL53L0X sensors at 0x30 and 0x31 (configured by pulling XSHUT pins low in sequence during initialization), and the VL53L1X at 0x32. If any sensor was missing or at a different address, we needed to catch that before writing code that assumes its presence.

## First Attempt: Direct smbus Access Without Error Handling

Our first version of the I2C scanner was naive. We simply looped through all possible addresses (0x03 to 0x77) and called `bus.read_byte(addr)` without any error handling. The smbus2 library raises an IOError when a read fails, which happens for addresses that have no device responding. Since the I2C bus has 120 possible addresses (from 0x03 to 0x7A, excluding reserved ranges) and we only have 5 devices, the vast majority of reads would fail. Without a try/except block, the first failed read would crash the entire script, giving us no information about which sensors were present.

The crash output was:

```
Traceback (most recent call last):
  File "tools/i2c_scan.py", line 8, in <module>
    bus.read_byte(0x03)
IOError: [Errno 5] Input/output error
```

Error 5 on Linux means "I/O error" — the kernel reported that the I2C transaction failed because no device acknowledged the address. This is expected behavior for an empty address, but our script was not prepared to handle it.

## The Fix: Try/Except Around Each Read

The fix was straightforward: wrap each `bus.read_byte()` call in a try/except block. If the read fails, we silently skip that address and continue scanning. We also added a lookup dictionary mapping known addresses to sensor names, so when a device is detected, we print its friendly name rather than just a hex address.

```python
try:
    bus.read_byte(addr)
    name = expected.get(addr, "Unknown")
    print(f"  0x{addr:02X} - {name} DETECTED")
except:
    pass  # No device at this address
```

This pattern — silently ignoring expected failures — is common in hardware scanning code. The alternative would be to check a device ID register, but that requires knowing the register address for each sensor, which adds complexity. For a simple presence check, reading any byte is sufficient: if the device ACKs the address phase, it is present.

## Alternative Approach: i2cdetect Command Line

We considered using the `i2cdetect -y 1` command line tool, which performs the same scan and prints a formatted table. The `i2cdetect` tool is part of the i2c-tools package on Raspberry Pi OS and works reliably. However, we chose to write our own Python scanner for several reasons.

First, we wanted programmatic access to the results. Our self-test script (developed later in v1.8) needs to check sensor presence automatically and report failures to a logging system. Parsing i2cdetect's text output would be fragile — its formatting could change between versions, and it prints different characters for different detection modes (UU for kernel driver, -- for empty, 0x address for detected). Our Python script returns results that can be used programmatically.

Second, we needed to test the smbus2 library before using it for actual sensor reading. The scanner serves as a smoke test for the I2C library itself. If smbus2 cannot even read a byte, there is no point trying to read multiple-byte registers for sensor data.

Third, writing our own scanner gave us full control over the output format. We could print sensor names alongside addresses, highlight missing sensors, and integrate the output into our logging system.

## The MPU6050 Responds But Gyro Not Yet Working

The scanner successfully detected the MPU6050 at 0x68, the QMC5883L at 0x0D, and the two VL53L0X sensors at 0x30 and 0x31. However, one of the VL53L0X sensors (the right one at 0x31) was not detected on the first few runs. We traced this to the initialization sequence: both VL53L0X sensors share the same default address (0x29) and must be assigned unique addresses by holding their XSHUT pins low, powering them one at a time, and sending a new address via I2C. Our wiring had the XSHUT pins connected to GPIO 16 and GPIO 17, but the initialization script had not been run yet. The scanner was detecting the first VL53L0X (the one that happened to power up first) at 0x29 (the default) rather than the expected 0x30.

We fixed this by adding a separate VL53L0X initialization step before scanning. This step brings both sensors out of reset one at a time, assigns addresses 0x30 and 0x31, and verifies the assignment by reading back the address. This initialization is now part of the boot sequence.

## The VL53L1X at 0x32

The VL53L1X front sensor was also not detected initially. Unlike the VL53L0X, the VL53L1X has a default address of 0x29 as well — the same as the VL53L0X! This is a known design conflict between STMicroelectronics sensors. If both a VL53L0X and a VL53L1X are on the same bus without address remapping, they will conflict at 0x29. Our solution was to initialize the VL53L1X first (since it has a different XSHUT pin on GPIO 18), assign it address 0x32, then initialize the two VL53L0X sensors to 0x30 and 0x31. The order of initialization matters and must be consistent.

## Learned: Always Check Bus Availability Before Reading

One issue that surfaced was that the I2C bus itself was not always available. If we ran the scanner immediately after boot, before the kernel had loaded the I2C driver, we would get a "No such file or directory" error when trying to open `/dev/i2c-1`. We added a check at the start of the script to verify that `/dev/i2c-1` exists and is readable. If not, the script prints a helpful error message suggesting to enable I2C via `raspi-config` or check the kernel module.

```python
import os
if not os.path.exists("/dev/i2c-1"):
    print("ERROR: I2C bus not found. Enable I2C with raspi-config.")
    sys.exit(1)
```

This simple check saved us debugging time on subsequent boots where we forgot to enable I2C.

## I2C Bus Speed Configuration

The Raspberry Pi 4's I2C bus runs at a default speed of 100kHz (standard mode). All five of our sensors support this speed, so we did not need to change it. However, for future optimization, the Pi supports 400kHz (fast mode) by adding `dtparam=i2c_arm=on,i2c_arm_baudrate=400000` to config.txt. Fast mode would reduce the time spent on I2C transactions, potentially allowing higher sensor read rates. We decided against enabling fast mode for now because the VL53L0X sensors have a maximum I2C speed of 400kHz but can exhibit timing issues with long cables at higher speeds. Our I2C cable length is approximately 15cm, which is within the recommended range for 400kHz, but we prefer reliability over speed during the development phase. If we need higher sensor throughput in later versions, we can enable fast mode and retest all sensors.

## Voltage Level Considerations

The Raspberry Pi 4's GPIO pins operate at 3.3V logic levels, but some I2C sensors use 5V logic. The MPU6050, QMC5883L, VL53L0X, and VL53L1X all operate at 3.3V, so this is not an issue for our sensor suite. However, we verified with an oscilloscope that the I2C bus signals (SDA and SCL) are clean square waves between 0V and 3.3V with no ringing or overshoot. The I2C pull-up resistors on the Pi's board (1.8kΩ to 3.3V) provide adequate rise times for both 100kHz and 400kHz operation. We measured the rise time at approximately 150ns, which is well within the I2C specification of 1000ns maximum for standard mode. If we were to extend the I2C cable beyond 30cm, we might need to add external pull-up resistors with lower values to maintain signal integrity. We have noted this in the hardware design document for future reference.

## Summary

The I2C scanner was our first real hardware test. It confirmed that all five sensors are present and responding, taught us about I2C error handling, and revealed the address conflict between VL53L0X and VL53L1X sensors. The scanner script would be reused in v1.8 as part of the self-test suite. We also established the pattern of creating dedicated test tools in the `tools/` directory, each with a single responsibility and clear output.
