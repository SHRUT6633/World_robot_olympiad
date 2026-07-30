import copy


class GainScheduler:
    def __init__(self, transition_band=0.1):
        self.transition_band = transition_band
        self._zones = [
            {"max_speed": 0.5, "gains": {"kp": 0.8, "ki": 0.05, "kd": 0.02}},
            {"max_speed": 1.0, "gains": {"kp": 0.5, "ki": 0.03, "kd": 0.01}},
            {"max_speed": 2.0, "gains": {"kp": 0.3, "ki": 0.01, "kd": 0.005}},
        ]
        self.current_zone = None

    def _interpolate(self, v, low_speed, high_speed, low_gains, high_gains):
        t = (v - low_speed) / (high_speed - low_speed)
        t = max(0.0, min(1.0, t))
        result = {}
        for k in low_gains:
            result[k] = low_gains[k] + t * (high_gains[k] - low_gains[k])
        return result

    def select(self, v):
        gains = self._zones[0]["gains"]
        prev_speed = 0.0
        prev_gains = self._zones[0]["gains"]
        self.current_zone = 0

        for i, zone in enumerate(self._zones):
            if v <= zone["max_speed"]:
                self.current_zone = i
                low = max(prev_speed, zone["max_speed"] - self.transition_band)
                high = zone["max_speed"]
                if v >= low:
                    return self._interpolate(v, low, high, prev_gains, zone["gains"])
                else:
                    return copy.deepcopy(prev_gains)
            prev_speed = zone["max_speed"]
            prev_gains = zone["gains"]

        self.current_zone = len(self._zones) - 1
        return copy.deepcopy(self._zones[-1]["gains"])
