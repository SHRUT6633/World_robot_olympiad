import numpy as np
from ..system.logger import log


class FeatureMap:
    def __init__(self):
        self.features = []
        self._next_id = 0

    def add(self, x, y, descriptor=None):
        feat = {"id": self._next_id, "x": x, "y": y, "descriptor": descriptor}
        self.features.append(feat)
        self._next_id += 1
        return feat["id"]

    def find_by_descriptor(self, des, threshold=50):
        best_dist = float("inf")
        best_id = None
        for feat in self.features:
            if feat["descriptor"] is not None and des is not None:
                d = np.linalg.norm(np.array(feat["descriptor"]) - np.array(des))
                if d < best_dist and d < threshold:
                    best_dist = d
                    best_id = feat["id"]
        return best_id

    def to_dict(self):
        return [{"id": f["id"], "x": f["x"], "y": f["y"]} for f in self.features]
