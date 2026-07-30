import numpy as np


class DynamicBicycleModel:
    # A 2D dynamic bicycle model with linear tire forces.
    # Unlike the kinematic model (which assumes no slip), this model accounts
    # for lateral tire forces and yaw inertia, making it accurate at higher
    # speeds where tire slip is significant.
    #
    # State vector: [x, y, psi, vx, vy, psi_dot]
    #   x, y: global position (m)
    #   psi: heading/yaw angle (rad)
    #   vx: longitudinal velocity (m/s) in the body frame
    #   vy: lateral velocity (m/s) in the body frame
    #   psi_dot: yaw rate (rad/s)

    def __init__(self, mass=2.0, Iz=0.1, lf=0.13, lr=0.13, Cf=50, Cr=50):
        # mass (m): total vehicle mass (kg).
        # Iz: yaw moment of inertia (kg·m²). Higher Iz = slower yaw response.
        # lf: distance from CG (center of gravity) to front axle (m).
        # lr: distance from CG to rear axle (m). lf + lr = wheelbase.
        # Cf: front tire cornering stiffness (N/rad). Higher = more lateral
        #     force per unit slip angle, i.e., grippier front tires.
        # Cr: rear tire cornering stiffness (N/rad).
        #
        # Changing Cf/Cr alters understeer/oversteer balance:
        #   Cf > Cr = understeer (car wants to go straight).
        #   Cf < Cr = oversteer (car rotates more aggressively).
        # Changing lf/lr moves the CG, affecting weight distribution.
        self.m = mass
        self.Iz = Iz
        self.lf = lf
        self.lr = lr
        self.Cf = Cf
        self.Cr = Cr

    def update(self, state, delta, dt):
        # state: current state vector (length-6 numpy array).
        # delta: front steering angle (rad).
        # dt: time step (s).
        # Returns: new state vector after dt seconds.
        #
        # --- Physical model ---
        # Tire slip angles (linearized with arctan):
        #   Front: alpha_f = arctan((vy + lf * psi_dot) / vx) - delta
        #   Rear:  alpha_r = arctan((vy - lr * psi_dot) / vx)
        # The 1e-6 prevents division by zero when vx ~ 0.
        #
        # Lateral tire forces (linear model):
        #   Fyf = -Cf * alpha_f
        #   Fyr = -Cr * alpha_r
        # (Negative sign because force opposes slip direction.)
        #
        # Longitudinal dynamics:
        #   vx_dot = 0 (no longitudinal forces modeled — assumes constant speed
        #            or ignores acceleration/deceleration for steering studies).
        #
        # Lateral dynamics (body-frame):
        #   vy_dot = (Fyf + Fyr) / m - vx * psi_dot
        #   (The -vx*psi_dot term is the centrifugal / Coriolis coupling.)
        #
        # Yaw dynamics:
        #   psi_ddot = (lf * Fyf - lr * Fyr) / Iz
        #
        # Kinematics (global frame update):
        #   x_dot = vx * cos(psi) - vy * sin(psi)
        #   y_dot = vx * sin(psi) + vy * cos(psi)
        #
        # All states are integrated forward with Euler integration.
        # Using a smaller dt improves accuracy. If dt is too large, the
        # integration becomes unstable (especially at high speeds).
        x, y, psi, vx, vy, psi_dot = state
        Fyf = -self.Cf * np.arctan((vy + self.lf * psi_dot) / (vx + 1e-6) - delta)
        Fyr = -self.Cr * np.arctan((vy - self.lr * psi_dot) / (vx + 1e-6))
        vx_dot = 0.0
        vy_dot = (Fyf + Fyr) / self.m - vx * psi_dot
        psi_ddot = (self.lf * Fyf - self.lr * Fyr) / self.Iz
        return np.array([
            x + (vx * np.cos(psi) - vy * np.sin(psi)) * dt,
            y + (vx * np.sin(psi) + vy * np.cos(psi)) * dt,
            psi + psi_dot * dt,
            vx + vx_dot * dt,
            vy + vy_dot * dt,
            psi_dot + psi_ddot * dt,
        ])
