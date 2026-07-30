import numpy as np


class KinematicModel:
    # Bicycle (Ackermann) kinematic model for a wheeled robot.
    #
    # This model assumes no wheel slip and represents the robot as a bicycle:
    # - L (wheelbase): distance between front and rear axles (meters).
    #   Smaller wheelbase -> tighter turning radius for the same steering angle.
    #   Typical value: 0.26 m for a small WRO robot.
    #
    # State: (x, y, heading)
    # Control: (v = forward speed, delta = steering angle)
    #
    # Equations (discrete-time, Euler integration):
    #   x_next     = x + v * cos(heading) * dt
    #   y_next     = y + v * sin(heading) * dt
    #   heading_next = heading + (v / L) * tan(delta) * dt
    #
    # Note: heading divergence is proportional to v / L. At low speed or large L,
    # the robot turns slowly; at high speed or small L, it turns quickly.

    def __init__(self, wheelbase=0.26):
        self.L = wheelbase   # Wheelbase in meters

    def update(self, x, y, heading, v, delta, dt):
        # Forward-simulate the bicycle model by one time step dt.
        x_next = x + v * np.cos(heading) * dt
        y_next = y + v * np.sin(heading) * dt
        heading_next = heading + (v / self.L) * np.tan(delta) * dt
        return x_next, y_next, heading_next

    def compute_steering(self, v, yaw_rate):
        # Inverse of the kinematic model: given a desired yaw_rate (rad/s) and speed v (m/s),
        # compute the required steering angle delta.
        # If speed is near zero, any steering would cause an instant heading change, so return 0.
        if abs(v) < 0.01:
            return 0.0
        return np.arctan(self.L * yaw_rate / v)
