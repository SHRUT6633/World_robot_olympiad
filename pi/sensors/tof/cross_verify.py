# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/tof/cross_verify.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Cross-verification of multi-ToF distance readings
# =============================================================================

from ...system.logger import log


class CrossSensorVerifier:
    """
    Cross-verification of distance readings from multiple ToF sensors.

    In a typical WRO robot, multiple ToF sensors are mounted around the
    chassis (front, left, right, back). When two or more sensors point
    in approximately the same direction (or when the robot faces a wall
    and multiple sensors see the wall), their readings should agree
    within a tolerance.

    This verifier:
      1. Checks each sensor reading against all others.
      2. If any pair disagrees by more than threshold_mm, marks both as
         inconsistent (or the one that deviates — currently marks the
         sensor under test).
      3. Computes a fused (averaged) distance from all consistent readings.

    Why this matters:
      - A sensor may be blocked by dirt, tape, or an object near the lens.
      - A sensor may have a hardware fault (e.g. damaged VCSEL).
      - In multi-path scenarios (e.g. glass wall), one sensor may give a
        wildly wrong reading while others are correct.

    Configuration:
      threshold_mm : Maximum allowed disagreement between any two sensors.
                     Set based on the expected variation from different
                     sensor positions on the chassis. For sensors 10 cm
                     apart, a 100 mm threshold is reasonable.
    """

    def __init__(self, threshold_mm=100):
        # Maximum pairwise difference (mm) before a sensor is flagged invalid.
        self.threshold = threshold_mm

    def verify(self, readings: dict) -> dict:
        """
        Cross-check all sensor readings against each other.

        readings: {sensor_name: distance_mm or None}
        returns:  {sensor_name: True (consistent) or False (inconsistent)}

        Algorithm:
          For each sensor i:
            - If reading is None, mark as invalid.
            - Otherwise, compare with every other sensor j:
              - If |reading_i - reading_j| > threshold_mm, mark i as
                inconsistent (and break inner loop — no need to check
                further).

        This is O(N^2) but N is typically 2–4, so it is fine.
        """
        valid = {}
        names = list(readings.keys())
        for i, name in enumerate(names):
            if readings[name] is None:
                # No data from this sensor — cannot be trusted.
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
        """
        Compute the average of all consistent, valid readings.

        This fused distance is safer to use in control logic than any
        single sensor's reading, because a single faulty sensor has
        limited impact.

        Returns None if NO readings are valid (robot should emergency stop
        or switch to a different sensing modality).
        """
        valid_vals = [v for n, v in readings.items() if v is not None and valid.get(n, False)]
        if not valid_vals:
            return None
        return sum(valid_vals) / len(valid_vals)
