import numpy as np


class MechanicalLinkage:
    # Models the mechanical linkage between the servo motor and the steering
    # wheels. In a real robot, the servo output angle is translated through
    # linkages (e.g., bell cranks, tie rods) to the wheel angle, which may
    # involve a gear ratio and physical steering limit.
    #
    # This class converts between servo angle and wheel angle, applying a
    # gear ratio and clipping to the maximum steering range.

    def __init__(self, gear_ratio=1.0, max_steering_deg=30):
        # gear_ratio: multiplier from servo angle to wheel angle.
        #   gear_ratio = 2.0 means the wheel moves twice as far as the servo.
        #   Changing this changes the mechanical advantage and steering sensitivity.
        # max_steering_deg: the physical maximum wheel angle (in degrees).
        #   Converted to radians internally. This is the mechanical stop limit.
        #   A larger value allows tighter turns but may cause mechanical binding.
        self.gear_ratio = gear_ratio
        self.max_steering = np.radians(max_steering_deg)

    def servo_to_wheel(self, servo_angle):
        # servo_angle: commanded servo angle (rad).
        # Returns: resulting wheel angle (rad), clipped to mechanical limits.
        #
        # First, multiply by gear_ratio to get the mechanical wheel angle.
        # Then clip to [-max_steering, max_steering] so the result respects
        # the physical steering stop.
        #
        # If gear_ratio is large, small servo changes produce large wheel
        # movements (more sensitive, harder to control precisely).
        # If gear_ratio is small, the robot's steering response is sluggish.
        wheel_angle = servo_angle * self.gear_ratio
        return np.clip(wheel_angle, -self.max_steering, self.max_steering)

    def wheel_to_servo(self, wheel_angle):
        # wheel_angle: desired wheel angle (rad).
        # Returns: the servo angle (rad) required to achieve it.
        #
        # Inverse of servo_to_wheel. Note: this does NOT check if the
        # wheel_angle exceeds the max steering limit or if the resulting
        # servo angle is within the servo's own range.
        # If wheel_angle is too large, the servo command may be infeasible
        # (the servo will saturate or the linkage will bind).
        return wheel_angle / self.gear_ratio
