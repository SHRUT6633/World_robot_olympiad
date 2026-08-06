import numpy as np


MATCH_PIXEL_THRESHOLD = 80
MAX_MISSED_FRAMES = 10
DT = 0.033


class KalmanTrack:
    def __init__(self, detection, track_id, dt=DT):
        self.id = track_id
        self.missed = 0
        self.dt = dt

        x, y = detection["center"]
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 100.0
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float64)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)
        self.Q = np.eye(4) * 0.1
        self.R = np.eye(2) * 10.0

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        z = np.array(z, dtype=np.float64)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.missed = 0

    def center(self):
        return (int(self.x[0]), int(self.x[1]))


class TrackPillars:
    def __init__(self):
        self.tracks = []
        self.next_id = 0

    def process(self, detections):
        for t in self.tracks:
            t.predict()
            t.missed += 1

        matched = [False] * len(detections)
        for t in self.tracks:
            best_d = None
            best_dist = MATCH_PIXEL_THRESHOLD
            for i, d in enumerate(detections):
                if matched[i]:
                    continue
                dist = np.linalg.norm(
                    np.array(t.center()) - np.array(d["center"])
                )
                if dist < best_dist:
                    best_dist = dist
                    best_d = i
            if best_d is not None:
                t.update(detections[best_d]["center"])
                matched[best_d] = True

        for i, m in enumerate(matched):
            if not m:
                t = KalmanTrack(detections[i], self.next_id)
                self.tracks.append(t)
                self.next_id += 1

        self.tracks = [t for t in self.tracks if t.missed < MAX_MISSED_FRAMES]

        return [
            {"id": t.id, "center": t.center(), "missed": t.missed}
            for t in self.tracks
        ]
