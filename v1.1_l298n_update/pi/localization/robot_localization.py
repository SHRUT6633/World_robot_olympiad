from .pose_estimation import PoseEstimator
from ..system.logger import log


class RobotLocalization:
    def __init__(self):
        self.pose = PoseEstimator()
        self._estimator = None

    def attach_filter(self, estimator):
        self._estimator = estimator

    def predict(self, dt):
        if self._estimator:
            self._estimator.predict(dt)

    def update(self, measurements):
        if self._estimator:
            self._estimator.update(measurements)
            state = self._estimator.state
            self.pose.update_absolute(state[0], state[1], state[2])

    def to_dict(self):
        return self.pose.to_dict()
