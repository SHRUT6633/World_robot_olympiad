# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/localization/robot_localization.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: High-level localisation manager with EKF/UKF filter pipeline
# =============================================================================

from .pose_estimation import PoseEstimator
from ..system.logger import log


class RobotLocalization:
    # High-level localisation manager that bridges a pose estimator with a
    # state estimation filter (EKF or UKF).
    #
    # The filter (EKF/UKF) handles prediction (motion model) and correction
    # (sensor measurements). After each update, the filter's state is written
    # into the underlying PoseEstimator, which owns the canonical pose data
    # used by the rest of the control system.

    def __init__(self):
        self.pose = PoseEstimator()  # Holds the current best pose estimate
        self._estimator = None       # EKF or UKF instance (assigned later via attach_filter)

    def attach_filter(self, estimator):
        # Attach a state estimator (EKF or UKF) to the pipeline.
        # The estimator must have .predict(dt), .update(z), and .state properties.
        self._estimator = estimator

    def predict(self, dt):
        # Run the filter's prediction step (motion model).
        # dt: time elapsed since last predict (seconds).
        if self._estimator:
            self._estimator.predict(dt)

    def update(self, measurements):
        # Run the filter's update step with new sensor measurements.
        # After the update, copy the filter's estimated state into the PoseEstimator.
        if self._estimator:
            self._estimator.update(measurements)
            state = self._estimator.state
            # state is [x, y, heading, v, a, yaw_rate] — copy the first 3 (pose)
            self.pose.update_absolute(state[0], state[1], state[2])

    def to_dict(self):
        # Serialise current pose to a dictionary for logging or telemetry.
        return self.pose.to_dict()
