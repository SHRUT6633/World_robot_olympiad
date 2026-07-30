import math
import time
import logging


CONFIDENCE_EVENTS = {
    "lap_completed": None,
    "corner_successful": 0.05,
    "obstacle_avoided": 0.03,
    "stuck_detected": -0.15,
    "emergency_stop": -0.30,
    "alignment_difficulty": -0.05,
}


class RaceStrategy:
    def __init__(self, min_speed=0.15, max_speed=0.35,
                 initial_confidence=0.5, smoothing_time=0.5):
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.initial_confidence = initial_confidence
        self.smoothing_time = smoothing_time

        self.confidence = initial_confidence
        self._current_speed = self._confidence_to_speed(initial_confidence)
        self.laps_completed = 0
        self._event_log = []
        self._last_speed_update = time.monotonic()

        self.logger = logging.getLogger(self.__class__.__name__)

    def _confidence_to_speed(self, confidence):
        clamped = max(0.0, min(1.0, confidence))
        speed_range = self.max_speed - self.min_speed
        return self.min_speed + clamped * speed_range

    def apply_event(self, event_name):
        if event_name not in CONFIDENCE_EVENTS:
            self.logger.warning(f"Unknown confidence event: {event_name}")
            return

        if event_name == "lap_completed":
            self.laps_completed += 1
            lap_bonus = 0.10 + 0.05 * self.laps_completed
            old_confidence = self.confidence
            self.confidence = min(1.0, self.confidence + lap_bonus)
            self._event_log.append({
                "time": time.time(),
                "event": event_name,
                "delta": lap_bonus,
                "old_confidence": old_confidence,
                "new_confidence": self.confidence,
            })
            self.logger.info(
                f"Lap {self.laps_completed} completed. "
                f"Confidence: {old_confidence:.2f} → {self.confidence:.2f} "
                f"(bonus: {lap_bonus:.2f})"
            )
            return

        delta = CONFIDENCE_EVENTS[event_name]
        old_confidence = self.confidence
        self.confidence = max(0.0, min(1.0, self.confidence + delta))
        self._event_log.append({
            "time": time.time(),
            "event": event_name,
            "delta": delta,
            "old_confidence": old_confidence,
            "new_confidence": self.confidence,
        })
        self.logger.debug(
            f"Event: {event_name} ({delta:+.2f}). "
            f"Confidence: {old_confidence:.2f} → {self.confidence:.2f}"
        )

    def get_target_speed(self):
        return self._confidence_to_speed(self.confidence)

    def get_smoothed_speed(self, dt=None):
        if dt is None:
            now = time.monotonic()
            dt = now - self._last_speed_update
            self._last_speed_update = now
            dt = max(0.001, min(dt, 0.1))

        target = self.get_target_speed()
        smoothing = 1.0 - math.exp(-dt / self.smoothing_time)
        self._current_speed += (target - self._current_speed) * smoothing
        return self._current_speed

    def get_confidence(self):
        return self.confidence

    def get_current_speed(self):
        return self._current_speed

    def get_laps_completed(self):
        return self.laps_completed

    def get_event_log(self):
        return list(self._event_log)

    def get_confidence_level(self):
        if self.confidence >= 0.8:
            return "high"
        elif self.confidence >= 0.4:
            return "medium"
        else:
            return "low"

    def is_confident(self):
        return self.confidence > 0.6

    def reset(self):
        self.confidence = self.initial_confidence
        self._current_speed = self._confidence_to_speed(self.initial_confidence)
        self.laps_completed = 0
        self._event_log.clear()
        self._last_speed_update = time.monotonic()
