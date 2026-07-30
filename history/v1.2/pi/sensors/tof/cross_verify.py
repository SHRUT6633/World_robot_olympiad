from ...system.logger import log


class CrossSensorVerifier:
    def __init__(self, threshold_mm=100):
        self.threshold = threshold_mm

    def verify(self, readings: dict) -> dict:
        valid = {}
        names = list(readings.keys())
        for i, name in enumerate(names):
            if readings[name] is None:
                valid[name] = False
                continue
            consistent = True
            for j, other in enumerate(names):
                if i == j or readings[other] is None:
                    continue
                if abs(readings[name] - readings[other]) > self.threshold:
                    consistent = False
                    break
            valid[name] = consistent
        return valid

    def fused_distance(self, readings: dict, valid: dict) -> float:
        valid_vals = [v for n, v in readings.items() if v is not None and valid.get(n, False)]
        if not valid_vals:
            return None
        return sum(valid_vals) / len(valid_vals)
