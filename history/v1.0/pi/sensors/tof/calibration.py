import json
from pathlib import Path
from ...system.logger import log


class ToFCalibration:
    def __init__(self):
        self.offsets = {}

    def calibrate_offset(self, sensor, true_distance_mm):
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
        return raw_mm - self.offsets.get(sensor_name, 0.0)

    def temp_compensate(self, raw_mm, temp_c, coeff=-0.03):
        return raw_mm + coeff * (temp_c - 25.0)

    def save(self, path="config/calibration/tof_calib.json"):
        Path(path).parent.mkdir(exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.offsets, f, indent=2)

    def load(self, path="config/calibration/tof_calib.json"):
        p = Path(path)
        if p.exists():
            with open(p) as f:
                self.offsets = json.load(f)
            return True
        return False
