# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/sensors/magnetometer/tilt_compensation.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Magnetometer tilt compensation
# =============================================================================

import numpy as np


class TiltCompensation:
    """
    Tilt compensation for magnetometer heading using accelerometer data.

    Physical principle:
      The Earth's magnetic field has a horizontal component (pointing
      toward magnetic north) and a vertical component (pointing into/
      out of the ground). The magnetometer measures the total field
      vector in its body-fixed frame.

      When the robot is level, the X-Y plane of the magnetometer is
      horizontal, and we can compute heading as:
        heading = arctan2(my, mx)

      When the robot tilts (pitch/roll ≠ 0), the magnetometer's axes
      are no longer aligned with the horizontal plane. Without correction,
      the vertical component of the Earth's field "leaks" into the X and Y
      readings, causing large heading errors — up to tens of degrees.

    How this corrects it:
      1. The accelerometer measures the gravity vector. Since gravity
         always points DOWN in the world frame, we can compute the
         robot's pitch and roll angles from the accel readings.
      2. Using these angles, we rotate the magnetic vector so that it
         is expressed in the horizontal (world) frame.
      3. The rotated X and Y components are then used for heading.

    Rotation model:
      We apply a two-step rotation:
        1. Rotate around Y by pitch (brings X into horizontal plane).
        2. Rotate around X by roll (brings Y into horizontal plane).

      The compensated X and Y are:
        X_h = mag_x * cos(pitch) + mag_z * sin(pitch)
        Y_h = mag_x * sin(roll) * sin(pitch) + mag_y * cos(roll)
              - mag_z * sin(roll) * cos(pitch)

    Usage:
      This class is used by the robot's navigation system. Before each
      heading calculation:
        1. Update orientation via update_orientation(accel).
        2. Call compensate(mag) to get tilt-corrected field vector.
      The compensated vector is then passed to heading().
    """

    def __init__(self):
        # Pitch angle in radians (rotation around Y axis).
        # Positive = nose up.
        self.pitch = 0.0
        # Roll angle in radians (rotation around X axis).
        # Positive = right side down.
        self.roll = 0.0

    def update_orientation(self, accel):
        """
        Compute pitch and roll from a 3-axis accelerometer reading.

        accel : numpy array (3,) — acceleration in g (gravity-compensated
                or raw; either works as long as the dominant component
                at rest is gravity).

        Pitch and roll are computed using the standard MEMS formulae:
          pitch = arcsin(-ax / |a|)
          roll  = arctan2(ay, az)

        Why -ax for pitch?
          When the robot pitches nose-up, the accelerometer X axis reads
          a component of gravity opposite to the tilt direction.
          Specifically, if the robot is pitched up by angle θ, the X axis
          reads -g * sin(θ). Hence pitch = arcsin(-ax / |a|).

        Why arctan2(ay, az) for roll?
          When the robot rolls, the Y and Z axes share the gravity
          component. arctan2(ay, az) gives the roll angle directly.

        The 1e-6 epsilon prevents division by zero (|a| = 0 in free fall,
        which should not happen in normal operation).
        """
        norm = np.linalg.norm(accel) + 1e-6
        self.pitch = np.arcsin(-accel[0] / norm)
        self.roll = np.arctan2(accel[1], accel[2])

    def compensate(self, mag):
        """
        Rotate the magnetometer reading from body frame to horizontal frame.

        mag : numpy array (3,) — raw magnetic field in body frame.

        Returns:
          numpy array (3,) — tilt-compensated field in horizontal frame.
          Z component is returned unchanged (not used for heading).

        Effect of changing pitch/roll:
          If the robot is on a slope (pitch = 10°), the uncompensated
          heading could be off by 15–20°. After compensation, the error
          is reduced to < 2°.
        """
        # Tilt-compensated X: project mag_x and mag_z onto horizontal X.
        x = mag[0] * np.cos(self.pitch) + mag[2] * np.sin(self.pitch)
        # Tilt-compensated Y: project all 3 axes onto horizontal Y.
        y = (mag[0] * np.sin(self.roll) * np.sin(self.pitch) +
             mag[1] * np.cos(self.roll) -
             mag[2] * np.sin(self.roll) * np.cos(self.pitch))
        return np.array([x, y, mag[2]])
