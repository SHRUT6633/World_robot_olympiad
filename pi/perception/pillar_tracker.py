import time
from ..system.logger import log


class PillarPassSide:
    LEFT = "LEFT"
    RIGHT = "RIGHT"


class PillarTracker:
    def __init__(self, pillar_logic="NORMAL"):
        self._pillar_logic = pillar_logic
        self._passed_pillars = []
        self._correct_count = 0
        self._wrong_count = 0
        self._pillar_cooldown = {}
        self._bearing_history = {}
        self._tracking_pillar = None

    def update(self, pillar_detections):
        if not pillar_detections:
            return

        now = time.time()
        for label, detections in pillar_detections.items():
            if label == "pink":
                continue
            if not detections:
                continue

            best = detections[0]
            pid = (label, best["x"], best["y"])
            cooldown_key = label

            last_time = self._pillar_cooldown.get(cooldown_key, 0)
            if now - last_time < 0.5:
                continue

            if best["distance_mm"] and best["distance_mm"] < 200:
                bearing = best["bearing"]
                side = PillarPassSide.LEFT if bearing < 0 else PillarPassSide.RIGHT
                correct_side_for_pillar = self._expected_correct_side(label)
                is_correct = side == correct_side_for_pillar

                self._passed_pillars.append({
                    "label": label,
                    "side": side,
                    "correct": is_correct,
                    "time": now,
                    "bearing": bearing,
                    "distance_mm": best["distance_mm"],
                })

                if is_correct:
                    self._correct_count += 1
                else:
                    self._wrong_count += 1

                self._pillar_cooldown[cooldown_key] = now
                log.info(
                    f"Pillar {label}: {'CORRECT' if is_correct else 'WRONG'} "
                    f"({side}) bearing={best['bearing']:.2f} "
                    f"dist={best['distance_mm']:.0f}mm"
                )

    def _expected_correct_side(self, label):
        if self._pillar_logic == "REVERSED":
            if label == "red":
                return PillarPassSide.RIGHT
            else:
                return PillarPassSide.LEFT
        else:
            if label == "red":
                return PillarPassSide.LEFT
            else:
                return PillarPassSide.RIGHT

    def correct_count(self):
        return self._correct_count

    def wrong_count(self):
        return self._wrong_count

    def total_passed(self):
        return len(self._passed_pillars)

    def last_pillar(self):
        if not self._passed_pillars:
            return None
        return self._passed_pillars[-1]

    def reset(self):
        self._passed_pillars.clear()
        self._correct_count = 0
        self._wrong_count = 0
        self._pillar_cooldown.clear()
