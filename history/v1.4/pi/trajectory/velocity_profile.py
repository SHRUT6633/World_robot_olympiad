import numpy as np


class VelocityProfiler:
    # VelocityProfiler computes a speed for each point along a path, creating
    # a velocity profile.  Two methods are provided:
    #   1. trapezoidal()  -- accelerate, cruise at max_v, decelerate.
    #   2. curvature_limited() -- reduce speed in curves to respect a
    #      maximum lateral acceleration.
    # The controller uses these profiles to set the target motor speed.

    def __init__(self, max_v=2.0, max_a=1.0):
        # max_v -- maximum linear velocity (m/s).  Increase for faster but
        # riskier driving; decrease for safer, more conservative motion.
        # max_a -- maximum linear acceleration (m/s^2).  Higher values give
        # snappier response but can cause wheel slip or jerk.
        self.max_v = max_v
        self.max_a = max_a

    def trapezoidal(self, path, dt=0.01):
        # Generate a trapezoidal velocity profile for the given path.
        # path -- array/list of waypoints (used only for length).
        # dt   -- expected control-loop timestep (seconds).
        # Returns an array of velocities, one per waypoint, clipped to max_v.

        n = len(path)
        v_profile = np.zeros(n)

        # Number of timesteps required to reach max_v from rest.
        accel_steps = int(self.max_v / (self.max_a * dt))

        for i in range(n):
            if i < accel_steps:
                # Acceleration phase.
                v_profile[i] = self.max_a * dt * i
            elif i > n - accel_steps:
                # Deceleration phase.
                v_profile[i] = self.max_a * dt * (n - i - 1)
            else:
                # Cruise phase at max_v.
                v_profile[i] = self.max_v

        return np.clip(v_profile, 0, self.max_v)

    def curvature_limited(self, path, curvature, max_lat_a=2.0):
        # Compute a velocity limited by path curvature to keep lateral
        # acceleration below max_lat_a.
        # curvature -- array of curvature values (1/m) at each path point.
        # max_lat_a -- maximum allowable lateral acceleration (m/s^2).
        # Returns an array of velocity limits, element-wise min with max_v.
        #
        # The formula v = sqrt(max_lat_a / |curvature|) comes from the
        # centripetal acceleration constraint: a_lat = v^2 * curvature.
        # Adding 1e-6 prevents division by zero on straight segments.
        v_curv = np.sqrt(max_lat_a / (curvature + 1e-6))
        return np.minimum(v_curv, self.max_v)
