import json
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints


def sweep_parameters():
    results = []
    for log_q in range(-6, -1):
        for log_r in range(-3, 1):
            Q = 10 ** log_q
            R = 10 ** log_r
            points = MerweScaledSigmaPoints(6, alpha=0.1, beta=2.0, kappa=0)
            ukf = UnscentedKalmanFilter(
                dim_x=6, dim_z=3, dt=0.02, fx=_fx, hx=_hx, points=points
            )
            ukf.x = np.zeros(6)
            ukf.P = np.eye(6) * 0.1
            ukf.Q = np.eye(6) * Q
            ukf.R = np.eye(3) * R

            errors = []
            for t in np.arange(0, 5.0, 0.02):
                ukf.dt = 0.02
                ukf.predict()
                if int(t / 0.2) % 5 == 0:
                    true = np.array([0.5 * np.sin(t * 0.5),
                                     0.5 * (1 - np.cos(t * 0.5)),
                                     t * 0.5])
                    ukf.update(true + np.random.normal(0, 0.05, 3))
                    err = np.hypot(ukf.x[0] - true[0], ukf.x[1] - true[1])
                    errors.append(err)
            rms = np.sqrt(np.mean(np.array(errors) ** 2))
            results.append((Q, R, rms))
            print(f"[SWEEP] Q=1e{log_q} R=1e{log_r} RMS={rms*100:.1f}cm")

    best = min(results, key=lambda x: x[2])
    print(f"[BEST] Q={best[0]} R={best[1]} RMS={best[2]*100:.1f}cm")


def _fx(state, dt, **kwargs):
    x, y, heading, speed, accel, yaw_rate = state
    heading += yaw_rate * dt
    speed += accel * dt
    x += speed * np.cos(heading) * dt
    y += speed * np.sin(heading) * dt
    return np.array([x, y, heading, speed, accel, yaw_rate])


def _hx(state):
    return state[:3]


if __name__ == "__main__":
    sweep_parameters()
