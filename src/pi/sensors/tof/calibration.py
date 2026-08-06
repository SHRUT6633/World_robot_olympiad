# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/tof/calibration.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# ToF sensor calibration and compensation
# =============================================================================

import json
from pathlib import Path
from ...system.logger import log


class ToFCalibration:
    """
    Calibration and environmental compensation for Time-of-Flight sensors.

    ToF sensors measure distance, but the raw reading is affected by:

      1. Offset (systematic bias):
         Due to the sensor package, cover glass, or optical window, the
         sensor may consistently over- or under-read by a fixed offset
         (e.g. +15 mm). This is corrected by measuring a known distance
         and computing the delta.

      2. Temperature drift:
         The VCSEL wavelength and SPAD sensitivity change with temperature.
         The dominant effect is ~ -0.03 mm/°C for the VL53L0X/L1X.
         As the sensor heats up (or the ambient temperature changes), the
         reading drifts. We linearise around 25 °C (room temperature).

      3. Target reflectivity:
         Dark surfaces return less light, which can bias the measurement
         (the VL53L0X/L1X use histograms to mitigate this, but a small
         bias remains). This calibration does NOT compensate for
         reflectivity — that requires per-surface characterisation.

    Calibration procedure:
      1. Place the sensor at a known distance (e.g. 100 mm) from a flat,
         neutral-grey target.
      2. Call calibrate_offset(sensor, true_distance_mm).
      3. The average of 30 readings minus the true distance gives the offset.

    Temperature compensation formula:
        compensated = raw + coeff * (T_current - 25.0)
        coeff ≈ -0.03 mm/°C for VL53L0X/L1X.

      Example: raw = 200 mm, T = 35 °C (10 °C above reference)
               compensated = 200 + (-0.03) * (35 - 25) = 200 - 0.3 = 199.7 mm.
      The drift is small but can matter in precision tasks.

    How the robot uses this:
      After calibration and temperature compensation, the distance readings
      are accurate to ± a few mm, enabling:
        - Precise wall-following at 50 mm distance.
        - Accurate gap measurement for WRO precision driving.
        - Reliable obstacle detection thresholds (no false positives due
          to temperature drift).

    Save / Load:
      Offsets are persisted to JSON so calibration is performed only once
      (or updated periodically).
    """

    def __init__(self):
        # Dictionary: {sensor_name: offset_mm}
        # Stores the per-sensor systematic offset (in mm).
        self.offsets = {}

    def calibrate_offset(self, sensor, true_distance_mm):
        """
        Measure the sensor's systematic offset at a known distance.

        Takes 30 readings from the sensor, averages them, and computes:
            offset = average_reading - true_distance_mm

        A positive offset means the sensor reads too far.
        A negative offset means the sensor reads too close.

        The sensor must be stationary and aimed at a flat, non-reflective
        surface at exactly true_distance_mm.
        """
        readings = []
        for _ in range(30):
            val = sensor.read()
            if val is not None:
                readings.append(val)
        if readings:
            avg = sum(readings) / len(readings)
            offset = avg - true_distance_mm
            self.offsets[sensor.name] = offset
            log.info(f"{sensor.name}: offset={offset:.1f}mm")

    def apply(self, sensor_name, raw_mm):
        # Subtract per-sensor offset (systematic bias from cover glass, etc.).
        return raw_mm - self.offsets.get(sensor_name, 0.0)

    def temp_compensate(self, raw_mm, temp_c, coeff=-0.03):
        # First-order temperature correction: D ≈ -0.03 mm/°C for VL53L0X/L1X
        # as VCSEL wavelength and SPAD sensitivity drift with temperature.
        return raw_mm + coeff * (temp_c - 25.0)

    def save(self, path="config/calibration/tof_calib.json"):
        """
        Save offset calibration to a JSON file.

        JSON format: {"sensor_name": offset_mm, ...}
        """
        Path(path).parent.mkdir(exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.offsets, f, indent=2)

    def load(self, path="config/calibration/tof_calib.json"):
        """
        Load previously saved offset calibration.

        Returns True if file existed and was loaded.
        """
        p = Path(path)
        if p.exists():
            with open(p) as f:
                self.offsets = json.load(f)
            return True
        return False
