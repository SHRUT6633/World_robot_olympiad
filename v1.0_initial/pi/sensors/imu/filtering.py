import numpy as np
from scipy import signal


class IMUFilter:
    def __init__(self, cutoff_low=5.0, cutoff_high=0.1, fs=100.0):
        self.fs = fs
        nyq = 0.5 * fs
        self._b_low, self._a_low = signal.butter(2, cutoff_low / nyq, btype="low")
        self._b_high, self._a_high = signal.butter(2, cutoff_high / nyq, btype="high")
        self._zi_accel = {"x": None, "y": None, "z": None}
        self._zi_gyro = {"x": None, "y": None, "z": None}

    def low_pass(self, data, axis="x", sensor="accel"):
        zi_key = f"{sensor}_{axis}"
        zi = getattr(self, f"_zi_{sensor}")[axis]
        if zi is None:
            zi = signal.lfilter_zi(self._b_low, self._a_low) * data
        filtered, zi = signal.lfilter(self._b_low, self._a_low, [data], zi=zi)
        getattr(self, f"_zi_{sensor}")[axis] = zi
        return filtered[0]

    def high_pass(self, data, axis="x", sensor="gyro"):
        zi = getattr(self, f"_zi_{sensor}")[axis]
        if zi is None:
            zi = signal.lfilter_zi(self._b_high, self._a_high) * data
        filtered, zi = signal.lfilter(self._b_high, self._a_high, [data], zi=zi)
        getattr(self, f"_zi_{sensor}")[axis] = zi
        return filtered[0]

    def filter_imu_data(self, accel, gyro):
        return {
            "accel": np.array([
                self.low_pass(accel[0], "x", "accel"),
                self.low_pass(accel[1], "y", "accel"),
                self.low_pass(accel[2], "z", "accel"),
            ]),
            "gyro": np.array([
                self.high_pass(gyro[0], "x", "gyro"),
                self.high_pass(gyro[1], "y", "gyro"),
                self.high_pass(gyro[2], "z", "gyro"),
            ]),
        }
