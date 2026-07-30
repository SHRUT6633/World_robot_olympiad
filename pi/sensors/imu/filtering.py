# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/imu/filtering.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# IIR digital filters for IMU data
# =============================================================================

import numpy as np
from scipy import signal


class IMUFilter:
    """
    IIR (Infinite Impulse Response) digital filters for IMU data.

    The MPU6050 raw accelerometer and gyroscope signals contain:
      - High-frequency noise from motor vibrations, PWM ripple, and
        mechanical resonance.
      - Low-frequency drift (gyro bias walk, temperature effects).

    This class applies:
      - Low-pass filter on accelerometer (cutoff_low = 5 Hz):
        Removes vibration noise while preserving gravity vector for
        tilt estimation. 5 Hz is chosen because the robot's tilt
        dynamics are below 2–3 Hz; vibration is above 20 Hz.

      - High-pass filter on gyroscope (cutoff_high = 0.1 Hz):
        Removes DC offset (bias) and very slow drift. The robot's
        turns last < 5 seconds, so 0.1 Hz cutoff corresponds to
        a ~10-second time constant — fast enough to remove bias
        drift without distorting turns.

    Filter type:
      2nd-order Butterworth (maximally flat passband, -12 dB/oct roll-off).
      Chosen for simplicity and stability. Biquad (second-order section)
      implemented via scipy.signal.lfilter with persistent state (zi)
      to avoid filter startup transients.

    State (zi):
      scipy's lfilter maintains a "filter memory" vector zi. If we reset
      zi every call, the filter output would have a transient for the
      first N samples. By storing zi per axis, we achieve continuous
      filtering across multiple read() calls.

    Configuration parameters:
      cutoff_low  : Low-pass corner frequency in Hz (default 5.0).
      cutoff_high : High-pass corner frequency in Hz (default 0.1).
      fs          : Sampling frequency in Hz (default 100.0).
                    Must match the actual IMU read rate.

    Effects of changing parameters:
      - Increasing cutoff_low (e.g. 10 Hz): More vibration passes through,
        tilt estimate becomes noisier but responds faster.
      - Decreasing cutoff_low (e.g. 2 Hz): Smoother tilt but significant
        phase lag (~150 ms at 2 Hz) — robot may overshoot when stopping.
      - Increasing cutoff_high (e.g. 1 Hz): Gyro bias drift removed faster,
        but slow turns (e.g. 5-second 90° turn) may be distorted.
      - Decreasing cutoff_high (e.g. 0.01 Hz): Better turn accuracy but
        slow bias drift is not removed — heading integration may accumulate
        error.

    How the robot uses this:
      After filtering, the accel data is used for:
        - Tilt-compensated magnetometer heading.
        - Gravity-aligned odometry (projecting accel onto horizontal plane).
      The gyro data is used for:
        - Turn-rate feedback in the PID controller.
        - Yaw angle integration (dead-reckoning).
    """

    def __init__(self, cutoff_low=5.0, cutoff_high=0.1, fs=100.0):
        """
        cutoff_low  : Low-pass cutoff frequency (Hz) for accel.
        cutoff_high : High-pass cutoff frequency (Hz) for gyro.
        fs          : Sampling rate (Hz). Must match the control loop rate.
        """
        self.fs = fs
        nyq = 0.5 * fs  # Nyquist frequency = fs/2.
        # 2nd-order Butterworth low-pass filter coefficients (b = numerator,
        # a = denominator). Used for accelerometer.
        self._b_low, self._a_low = signal.butter(2, cutoff_low / nyq, btype="low")
        # 2nd-order Butterworth high-pass filter coefficients. Used for gyroscope.
        self._b_high, self._a_high = signal.butter(2, cutoff_high / nyq, btype="high")
        # Filter state (zi) for each axis of each sensor.
        # Initially None; will be initialised on first call.
        self._zi_accel = {"x": None, "y": None, "z": None}
        self._zi_gyro = {"x": None, "y": None, "z": None}

    def low_pass(self, data, axis="x", sensor="accel"):
        """
        Apply a 2nd-order Butterworth low-pass filter to a single axis value.

        Uses scipy.signal.lfilter with persistent state (zi) for
        continuous filtering. The zi is initialised assuming the input
        is in steady-state (zi = lfilter_zi * data) to avoid an initial
        transient spike.

        data  : single float value from the sensor.
        axis  : "x", "y", or "z" (selects which zi state to use).
        sensor: "accel" or "gyro" (selects which filter coefficients to use).

        Returns filtered float.
        """
        zi_key = f"{sensor}_{axis}"
        zi = getattr(self, f"_zi_{sensor}")[axis]
        if zi is None:
            # Initialise filter state assuming constant input.
            zi = signal.lfilter_zi(self._b_low, self._a_low) * data
        filtered, zi = signal.lfilter(self._b_low, self._a_low, [data], zi=zi)
        # Store updated state for next call.
        getattr(self, f"_zi_{sensor}")[axis] = zi
        return filtered[0]

    def high_pass(self, data, axis="x", sensor="gyro"):
        """
        Apply a 2nd-order Butterworth high-pass filter.

        High-pass removes DC offset and very slow drift.
        """
        zi = getattr(self, f"_zi_{sensor}")[axis]
        if zi is None:
            zi = signal.lfilter_zi(self._b_high, self._a_high) * data
        filtered, zi = signal.lfilter(self._b_high, self._a_high, [data], zi=zi)
        getattr(self, f"_zi_{sensor}")[axis] = zi
        return filtered[0]

    def filter_imu_data(self, accel, gyro):
        """
        Convenience method: filter all 6 axes at once.

        accel : numpy array (3,) — raw acceleration in g.
        gyro  : numpy array (3,) — raw angular velocity in °/s.

        Returns dict with filtered "accel" and "gyro" arrays.
        """
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
