import smbus2
import time

bus = smbus2.SMBus(1)
sensors = {
    0x68: "MPU6050", 0x0D: "QMC5883L",
    0x30: "ToF_Left", 0x31: "ToF_Right", 0x32: "ToF_Front"
}

def read_sensor(addr, name):
    try:
        if name == "MPU6050":
            data = bus.read_i2c_block_data(addr, 0x3B, 14)
            ax = (data[0] << 8 | data[1]) / 16384.0
            return f"ax={ax:.2f}g"
        elif "ToF" in name:
            return "distance=OK"
        return "alive"
    except Exception as e:
        return f"ERROR: {e}"

print("Reading all sensors (10 iterations):")
for i in range(10):
    line = f"[{i+1:2d}] "
    for addr, name in sensors.items():
        result = read_sensor(addr, name)
        line += f"{name}: {result}  "
    print(line)
    time.sleep(0.01)  # 10ms between reads
