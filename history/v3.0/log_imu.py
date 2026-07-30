import smbus2
import csv
import time
import numpy as np

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE = 16384.0  # LSB/g at +/-2g
GYRO_SCALE = 131.0     # LSB/deg/s at +/-250 deg/s

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)  # wake up

time.sleep(1.0)

for i in range(100):
    data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)

DURATION = 300  # seconds
SAMPLE_INTERVAL = 0.01  # 100 Hz target
BUFFER_SIZE = int(DURATION / SAMPLE_INTERVAL) + 1

log = np.zeros((BUFFER_SIZE, 8))  # t, ax, ay, az, gx, gy, gz
count = 0
start = time.perf_counter()
next_sample = start

while count < BUFFER_SIZE:
    now = time.perf_counter()
    if now < next_sample:
        continue

    try:
        data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
    except OSError as e:
        print(f"I2C error at sample {count}: {e}")
        continue

    ax = np.int16((data[0] << 8) | data[1]) / ACCEL_SCALE
    ay = np.int16((data[2] << 8) | data[3]) / ACCEL_SCALE
    az = np.int16((data[4] << 8) | data[5]) / ACCEL_SCALE
    gx = np.int16((data[8] << 8) | data[9]) / GYRO_SCALE
    gy = np.int16((data[10] << 8) | data[11]) / GYRO_SCALE
    gz = np.int16((data[12] << 8) | data[13]) / GYRO_SCALE

    log[count] = [now - start, ax, ay, az, gx, gy, gz]
    count += 1
    next_sample += SAMPLE_INTERVAL

with open("imu_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"])
    w.writerows(log[:count])

bus.close()
print(f"Logged {count} samples to imu_log.csv")
