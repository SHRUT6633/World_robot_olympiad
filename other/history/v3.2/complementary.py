import numpy as np
import json

class ComplementaryFilter:
    def __init__(self, alpha=0.92):
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, ax, ay, az, gx, gy, gz, dt):
        gx_rad = np.deg2rad(gx)
        gy_rad = np.deg2rad(gy)
        gz_rad = np.deg2rad(gz)

        accel_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm > 0.5 and accel_norm < 1.5:
            ax_n = ax / accel_norm
            ay_n = ay / accel_norm
            az_n = az / accel_norm

            accel_pitch = np.arctan2(-ax_n, np.sqrt(ay_n*ay_n + az_n*az_n))
            accel_roll = np.arctan2(ay_n, az_n)

            half_dt = 0.5 * dt
            q_dot = 0.5 * self._quat_multiply(self.q, [0, gx_rad, gy_rad, gz_rad])
            q_gyro = self.q + q_dot * dt
            q_gyro /= np.linalg.norm(q_gyro)

            q_accel = self._euler_to_quat(accel_pitch, accel_roll, 0.0)

            self.q = self._quat_slerp(q_gyro, q_accel, 1.0 - self.alpha)
            self.q /= np.linalg.norm(self.q)
        else:
            q_dot = 0.5 * self._quat_multiply(self.q, [0, gx_rad, gy_rad, gz_rad])
            self.q = self.q + q_dot * dt
            self.q /= np.linalg.norm(self.q)

        self.pitch, self.roll, _ = self._quat_to_euler(self.q)
        return self.pitch, self.roll

    def _quat_multiply(self, a, b):
        return np.array([
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
        ])

    def _quat_slerp(self, q1, q2, t):
        dot = np.dot(q1, q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        theta = np.arccos(np.clip(dot, -1.0, 1.0))
        if theta < 1e-10:
            return q1 + t * (q2 - q1)
        return (np.sin((1-t)*theta)*q1 + np.sin(t*theta)*q2) / np.sin(theta)

    def _euler_to_quat(self, pitch, roll, yaw):
        cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5)
        cr, sr = np.cos(roll*0.5), np.sin(roll*0.5)
        cy, sy = np.cos(yaw*0.5), np.sin(yaw*0.5)
        return np.array([
            cp*cr*cy + sp*sr*sy,
            sp*cr*cy - cp*sr*sy,
            cp*sr*cy + sp*cr*sy,
            cp*cr*sy - sp*sr*cy,
        ])

    def _quat_to_euler(self, q):
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return pitch, roll, yaw

if __name__ == "__main__":
    with open("imu_calib.json") as f:
        calib = json.load(f)
    gb = calib["gyro_bias"]
    filt = ComplementaryFilter(alpha=0.92)
    print("Complementary filter ready. Pitch/Roll at 95Hz.")
