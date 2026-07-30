# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/pillar_tracker.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Pillar passing tracker and logic
# =============================================================================

import time
from ..system.logger import log


class PillarPassSide:
    LEFT = "LEFT"
    RIGHT = "RIGHT"


class PillarTracker:
    # PillarTracker consumes the raw detections from PillarDetector and
    # decides whether each pillar was passed on the correct side according
    # to the WRO 2026 rules (red → pass left, green → pass right, or
    # reversed in the REVERSED logic variant).
    #
    # It applies a cooldown so that the same pillar is not double-counted,
    # and only registers a pass when the robot is close enough (distance <
    # 200 mm) to be sure it is actually passing, not just glimpsing it
    # from afar.

    def __init__(self, pillar_logic="NORMAL"):
        # pillar_logic: "NORMAL" → red=LEFT, green=RIGHT.
        #               "REVERSED" → red=RIGHT, green=LEFT.
        self._pillar_logic = pillar_logic
        self._passed_pillars = []    # History of all pillar-pass events.
        self._correct_count = 0      # How many were passed on the correct side.
        self._wrong_count = 0        # How many were passed on the wrong side.
        self._pillar_cooldown = {}   # Per-label timestamp to prevent double-count.
        self._bearing_history = {}
        self._tracking_pillar = None

    def update(self, pillar_detections):
        # pillar_detections: dict returned by PillarDetector.detect().
        #   Keys are "red", "green", "pink".  Only red and green are actual
        #   WRO score pillars; pink markers are ignored here.
        if not pillar_detections:
            return

        now = time.time()
        for label, detections in pillar_detections.items():
            # Pink markers are parking-zone markers, not scored pillars.
            if label == "pink":
                continue
            if not detections:
                continue

            # The first detection is the largest (sorted by PillarDetector).
            best = detections[0]
            pid = (label, best["x"], best["y"])
            cooldown_key = label

            # --- Cooldown to reject false-positive re-detections ---
            # After a pillar is registered, ignore the same colour for 0.5 s.
            # This prevents the tracker from counting the same pillar twice
            # as the robot drives past it.
            last_time = self._pillar_cooldown.get(cooldown_key, 0)
            if now - last_time < 0.5:
                continue

            # --- Distance gate: only count pillars that are very close ---
            # When the distance estimate drops below 200 mm, the robot is
            # considered to be "passing" the pillar.  This gates out
            # false triggers from pillars that are far away or in the
            # background.
            if best["distance_mm"] and best["distance_mm"] < 200:
                bearing = best["bearing"]

                # Determine which side of the image centre the pillar is on.
                side = PillarPassSide.LEFT if bearing < 0 else PillarPassSide.RIGHT

                # Compare actual side against the rulebook expectation.
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
        # WRO 2026 rulebook: red pillars should be passed on the LEFT,
        # green pillars on the RIGHT.  The REVERSED variant swaps them
        # (used for certain challenge runs or alternate courses).
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
