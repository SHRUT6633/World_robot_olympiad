import smbus2
import time
import json
import numpy as np

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0
N_SAMPLES = 1000
TEMP_THRESHOLD = 5.0

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
time.sleep(1.0)

def read_sensors():
    data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
    raw_temp = np.int16((data[6] << 8) | data[7])
    temp = raw_temp / 340.0 + 36.53
    ax = np.int16((data[0] << 8) | data[1]) / ACCEL_SCALE
    ay = np.int16((data[2] << 8) | data[3]) / ACCEL_SCALE
    az = np.int16((data[4] << 8) | data[5]) / ACCEL_SCALE
    gx = np.int16((data[8] << 8) | data[9]) / GYRO_SCALE
    gy = np.int16((data[10] << 8) | data[11]) / GYRO_SCALE
    gz = np.int16((data[12] << 8) | data[13]) / GYRO_SCALE
    return ax, ay, az, gx, gy, gz, temp

print("Collecting 1000 stationary samples for calibration...")
gx_samples, gy_samples, gz_samples = [], [], []
ax_samples, ay_samples, az_samples = [], [], []

for i in range(N_SAMPLES):
    ax, ay, az, gx, gy, gz, temp = read_sensors()
    gx_samples.append(gx)
    gy_samples.append(gy)
    gz_samples.append(gz)
    ax_samples.append(ax)
    ay_samples.append(ay)
    az_samples.append(az)
    time.sleep(0.01)

gyro_bias = {
    "x": float(np.mean(gx_samples)),
    "y": float(np.mean(gy_samples)),
    "z": float(np.mean(gz_samples)),
}
accel_scale = {
    "x": 1.0 / abs(float(np.mean(ax_samples))) if abs(np.mean(ax_samples)) > 0.1 else 1.0,
    "y": 1.0 / abs(float(np.mean(ay_samples))) if abs(np.mean(ay_samples)) > 0.1 else 1.0,
    "z": 1.0 / abs(float(np.mean(az_samples))) if abs(np.mean(az_samples)) > 0.1 else 1.0,
}
calib_temp = float(temp)

calib = {
    "gyro_bias": gyro_bias,
    "accel_scale": accel_scale,
    "calib_temp": calib_temp,
    "timestamp": time.time(),
}

with open("imu_calib.json", "w") as f:
    json.dump(calib, f, indent=2)

print(f"Gyro bias (deg/s): {gyro_bias}")
print(f"Accel scale: {accel_scale}")
print(f"Calibration temp: {calib_temp:.1f}C")

def check_temp_drift():
    while True:
        time.sleep(5.0)
        _, _, _, _, _, _, temp = read_sensors()
        if abs(temp - calib_temp) > TEMP_THRESHOLD:
            print(f"WARNING: Temp changed from {calib_temp:.1f}C to {temp:.1f}C")
            print("Recalibration recommended")

import threading
t = threading.Thread(target=check_temp_drift, daemon=True)
t.start()

bus.close()
