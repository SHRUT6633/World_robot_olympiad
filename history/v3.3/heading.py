import smbus2
import time
import numpy as np
import json

QMC_ADDR = 0x0D
MODE_CONTROL = 0x09
DATA_X_L = 0x00

bus = smbus2.SMBus(1)

X_OFFSET = 112.0
Y_OFFSET = -85.0

def calibrate_mag():
    print("Rotate robot 360 degrees slowly...")
    xs, ys = [], []
    start = time.time()
    while time.time() - start < 30.0:
        data = bus.read_i2c_block_data(QMC_ADDR, DATA_X_L, 6)
        x = np.int16((data[1] << 8) | data[0])
        y = np.int16((data[3] << 8) | data[2])
        z = np.int16((data[5] << 8) | data[4])
        xs.append(x)
        ys.append(y)
        time.sleep(0.05)
    x_off = (max(xs) + min(xs)) / 2
    y_off = (max(ys) + min(ys)) / 2
    calib = {"x_offset": x_off, "y_offset": y_off}
    with open("mag_calib.json", "w") as f:
        json.dump(calib, f, indent=2)
    print(f"Calibration: x_off={x_off:.1f}, y_off={y_off:.1f}")
    return calib

def read_heading():
    data = bus.read_i2c_block_data(QMC_ADDR, DATA_X_L, 6)
    x_raw = np.int16((data[1] << 8) | data[0])
    y_raw = np.int16((data[3] << 8) | data[2])

    x_corr = x_raw - X_OFFSET
    y_corr = y_raw - Y_OFFSET

    heading = (np.arctan2(-y_corr, x_corr) * 180.0 / np.pi) % 360.0
    return heading

if __name__ == "__main__":
    try:
        with open("mag_calib.json") as f:
            cal = json.load(f)
            X_OFFSET = cal["x_offset"]
            Y_OFFSET = cal["y_offset"]
    except FileNotFoundError:
        cal = calibrate_mag()
        X_OFFSET = cal["x_offset"]
        Y_OFFSET = cal["y_offset"]

    bus.write_byte_data(QMC_ADDR, MODE_CONTROL, 0x1D)  # 100Hz, OS512
    time.sleep(0.01)

    for _ in range(50):
        h = read_heading()
        print(f"Heading: {h:.1f} deg")
        time.sleep(0.01)
    bus.close()
