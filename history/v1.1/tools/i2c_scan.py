import smbus2
import sys

bus = smbus2.SMBus(1)  # I2C bus 1 on Pi 4
expected = {0x68: "MPU6050 IMU", 0x0D: "QMC5883L Mag",
            0x30: "VL53L0X Left", 0x31: "VL53L0X Right", 0x32: "VL53L1X Front"}

print("Scanning I2C bus...")
for addr in range(0x03, 0x78):
    try:
        bus.read_byte(addr)
        name = expected.get(addr, "Unknown")
        print(f"  0x{addr:02X} - {name} DETECTED")
    except:
        pass  # No device at this address
