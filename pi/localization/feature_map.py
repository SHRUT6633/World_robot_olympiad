# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/localization/feature_map.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Landmark feature storage for re-localisation and loop-closure detection
# =============================================================================

import numpy as np
from ..system.logger import log


class FeatureMap:
    # ──────────────────────────────────────────────────────────────────
    # Stores a list of landmark *features* (e.g. corners, QR codes,
    # ArUco markers).  Each feature has a unique ID, a position, and
    # an optional descriptor for appearance-based matching.
    #
    # Used for re-localisation and loop-closure detection.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        # features – list of dicts with keys: id, x, y, descriptor.
        self.features = []
        # Auto-incrementing ID counter.
        self._next_id = 0

    def add(self, x, y, descriptor=None):
        # Insert a new feature into the map.
        # Returns the newly-assigned integer ID.
        feat = {"id": self._next_id, "x": x, "y": y, "descriptor": descriptor}
        self.features.append(feat)
        self._next_id += 1
        return feat["id"]

    def find_by_descriptor(self, des, threshold=50):
        # Brute-force nearest-neighbour search over *descriptor*
        # space (e.g. 128-D SURF / ORB vector).
        #
        #   des – descriptor of the observed feature.
        #   threshold – maximum L2 distance for a match.
        #
        # Returns the ID of the closest match, or None.

        best_dist = float("inf")
        best_id = None

        for feat in self.features:
            # Skip features with no descriptor, or if query is None.
            if feat["descriptor"] is not None and des is not None:
                d = np.linalg.norm(
                    np.array(feat["descriptor"]) - np.array(des)
                )
                if d < best_dist and d < threshold:
                    best_dist = d
                    best_id = feat["id"]

        return best_id

    def to_dict(self):
        # Serialise the map (id + position) for saving / sending.
        # (Descriptors are omitted because they are often large.)
        return [{"id": f["id"], "x": f["x"], "y": f["y"]} for f in self.features]

# ── What happens if you change key values? ─────────────────────────
# * threshold  ↑ → more matches found but more false positives.
#   ↓ → only very strong matches accepted, may miss true positives.
# * If you remove the descriptor check `feat["descriptor"] is not None`
#   you could get type errors; the guard is important.
# * `to_dict` strips descriptors – if you need full persistence you
#   should extend this to also serialise the descriptor array.
# * This is a flat list; for large maps you'd want a spatial index
#   (e.g. KD-Tree) for efficient lookup.
# ────────────────────────────────────────────────────────────────────
