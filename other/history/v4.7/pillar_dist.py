import math


PILLAR_REAL_HEIGHT_MM = 100.0
FOCAL_LENGTH_PX = 520.0
FRAME_HEIGHT = 480


class PillarDistEstimate:
    def estimate_distance(self, pixel_height, pitch_rad):
        if pixel_height <= 0:
            return None
        corrected_h = pixel_height / max(math.cos(pitch_rad), 0.1)
        distance = (PILLAR_REAL_HEIGHT_MM * FOCAL_LENGTH_PX) / corrected_h
        return distance

    def is_reliable(self, bottom_y, pitch_rad):
        horizon_y = FRAME_HEIGHT / 2.0 + pitch_rad * FOCAL_LENGTH_PX
        return bottom_y > horizon_y

    def process(self, pillars, pitch_rad):
        for p in pillars:
            pixel_h = p.get("pixel_height", 0)
            if pixel_h == 0:
                box = p.get("box_points", None)
                if box and len(box) >= 4:
                    ys = [pt[1] for pt in box]
                    pixel_h = max(ys) - min(ys)
                else:
                    continue

            distance = self.estimate_distance(pixel_h, pitch_rad)
            bottom_y = p.get("bottom_y", 0)
            reliable = self.is_reliable(bottom_y, pitch_rad)

            p["distance_mm"] = distance
            p["distance_reliable"] = reliable

        return pillars
