# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/control/feedforward_comp.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Feedforward compensator for longitudinal acceleration control
# =============================================================================

import numpy as np


class FeedforwardCompensation:
    # A feedforward compensator for longitudinal (acceleration) control.
    # Instead of waiting for feedback to react, the feedforward path pre-computes
    # the force/torque needed based on a simple physics model:
    #   Force = mass * acceleration + friction * velocity
    #
    # This improves response time because the feedback loop only needs to correct
    # for model inaccuracies and disturbances instead of generating the entire
    # control signal.

    def __init__(self, mass=2.0, friction=0.1):
        # mass: estimated mass of the robot (kg).
        #       If larger than reality, the feedforward term over-estimates
        #       required force, causing aggressive acceleration.
        # friction: viscous friction coefficient (N·s/m).
        #           If too high, the controller compensates more for drag,
        #           potentially causing overshoot at low speeds.
        self.mass = mass
        self.friction = friction

    def compute(self, target_accel, v):
        # target_accel: desired acceleration (m/s^2).
        # v: current velocity (m/s).
        # Returns: required control effort (e.g., motor command).
        #
        # The formula F = m*a + friction*v models the force needed to achieve
        # the desired acceleration while overcoming viscous drag.
        # Changing mass or friction changes the estimated plant dynamics.
        # If the model is accurate, the feedback controller has very little
        # correction to do. If inaccurate, the feedback must compensate.
        # Newton's second law with linear viscous drag: F = m·a + b·v
        # mass and friction are lumped-parameter estimates of the real plant
        # Accuracy of these parameters directly determines how much the feedback
        # controller must compensate for — a well-tuned model minimises feedback effort
        return self.mass * target_accel + self.friction * v
