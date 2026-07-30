# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/localization/pose_graph.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Simple pose-graph (Graph SLAM) optimisation
# =============================================================================

import numpy as np
from ..system.logger import log


class PoseGraphOptimizer:
    # ──────────────────────────────────────────────────────────────────
    # Simple pose-graph (Graph SLAM) optimiser.
    #
    # The robot stores a sequence of *poses* and spatial *constraints*
    # between pairs of poses (e.g. from odometry or loop closures).
    # The optimiser iteratively adjusts the poses to minimise the
    # error introduced by these constraints, reducing drift.
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        # poses       – list of [x, y, heading] arrays.
        self.poses = []
        # constraints – list of tuples:
        #   (i, j, relative_transform, information_matrix)
        #   meaning "pose_j ≈ pose_i ⊕ relative_transform".
        self.constraints = []

    def add_pose(self, x, y, heading):
        # Append a new robot pose to the graph.
        self.poses.append(np.array([x, y, heading]))

    def add_constraint(self, i, j, dx, dy, dheading, info=np.eye(3)):
        # Add a relative constraint between pose[i] and pose[j].
        #   i,j        – indices into self.poses.
        #   dx,dy,dheading – measured relative transform.
        #   info       – 3×3 information matrix (inverse covariance),
        #                defaults to identity (equal weight on all
        #                components).
        self.constraints.append((i, j, np.array([dx, dy, dheading]), info))

    def optimize(self, iterations=10):
        # Iteratively adjust poses to satisfy the constraints.
        # If fewer than 2 poses, there is nothing to optimise.

        if len(self.poses) < 2:
            return self.poses

        # Work on a copy so we don't corrupt the originals mid-way.
        poses = [p.copy() for p in self.poses]

        for _ in range(iterations):
            for i, j, rel, info in self.constraints:
                # Guard against indices that have grown out of range.
                if i >= len(poses) or j >= len(poses):
                    continue

                # Error = measured - predicted = (pose_j - pose_i) - rel.
                error = poses[j] - poses[i] - rel

                # Normalise heading error to [-π, π] so we don't
                # accumulate unnecessary angular wraps.
                error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))

                # Simple gradient-descent step:
                #   pose_j ← pose_j - 0.1 * info @ error
                # The 0.1 is a fixed learning rate.
                correction = 0.1 * info @ error
                poses[j] -= correction

        self.poses = poses
        return self.poses

# ── What happens if you change key values? ─────────────────────────
# * iterations  ↑ → more accurate (converges further), slower.
#   ↓ → faster but may not converge fully.
# * learning rate (0.1)  ↑ → faster convergence but risk of
#   divergence / oscillation; ↓ → more stable but slower.
# * info matrix – scaling each constraint's influence.
#   E.g. give a loop-closure constraint much higher info than
#   odometry so it pulls the graph harder.
# * This is a *very* simplified optimiser.  Real systems use
#   Levenberg-Marquardt or g2o / GTSAM back-ends with robust
#   kernels to handle outlier constraints.
# * Only pose_j is adjusted (one-directional).  A proper graph
#   optimiser would split the correction across both poses.
# ────────────────────────────────────────────────────────────────────
