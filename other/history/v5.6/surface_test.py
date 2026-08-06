import numpy as np
from adaptive_noise import AdaptiveUKF


def simulate_surface(slip_factor: float, steps: int = 500):
    ukf = AdaptiveUKF()
    true_state = np.zeros(6)
    errors = []
    for i in range(steps):
        dt = 0.02
        control = np.array([0.3, 0.0])
        true_state[3] = control[0] + np.random.normal(0, slip_factor * 0.1)
        true_state[2] += control[1] * dt
        true_state[0] += true_state[3] * np.cos(true_state[2]) * dt
        true_state[1] += true_state[3] * np.sin(true_state[2]) * dt
        ukf.predict(dt)
        if i % 5 == 0:
            z = true_state[:3] + np.random.normal(0, 0.05, 3)
            ukf.correct(z)
            errors.append(np.hypot(ukf.ukf.x[0] - true_state[0],
                                   ukf.ukf.x[1] - true_state[1]))
    rms = np.sqrt(np.mean(np.array(errors) ** 2))
    final_q = np.trace(ukf.ukf.Q[:3, :3])
    print(f"[SURFACE] slip={slip_factor:.1f} Q_trace={final_q:.5f} "
          f"RMS={rms*100:.1f}cm")
    return rms


if __name__ == "__main__":
    simulate_surface(0.3)
    simulate_surface(1.0)
