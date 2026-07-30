def test_gain_transitions():
    sched = GainScheduler(transition_band=0.1)

    print("Testing gain interpolation across speed range\n")

    prev_kp = None
    for v in [x * 0.02 for x in range(0, 101)]:
        g = sched.select(v)
        kp = g["kp"]
        ki = g["ki"]
        kd = g["kd"]

        status = ""
        if prev_kp is not None:
            dkp = abs(kp - prev_kp)
            if dkp > 0.03:
                status = f" *** WARNING: kp jump = {dkp:.3f}"
        prev_kp = kp

        if abs(v - round(v * 10) / 10) < 0.001:
            print(f"v={v:.1f}  kp={kp:.4f}  ki={ki:.4f}  kd={kd:.4f}{status}")

    print("\n--- Zone transition checks ---")

    g_slow = sched.select(0.3)
    g_med = sched.select(0.7)
    g_fast = sched.select(1.5)
    print(f"At 0.3 m/s (slow zone):  kp={g_slow['kp']:.3f}")
    print(f"At 0.7 m/s (med zone):   kp={g_med['kp']:.3f}")
    print(f"At 1.5 m/s (fast zone):  kp={g_fast['kp']:.3f}")

    assert g_slow["kp"] > g_med["kp"] > g_fast["kp"], "kp should decrease with speed"
    assert g_slow["ki"] > g_med["ki"] > g_fast["ki"], "ki should decrease with speed"
    assert g_slow["kd"] > g_med["kd"] > g_fast["kd"], "kd should decrease with speed"
    print("\nAll assertions passed.")


def test_transition_continuity():
    sched = GainScheduler(transition_band=0.05)
    prev_kp = sched.select(0.0)["kp"]
    max_jump = 0.0

    for v in [x * 0.001 for x in range(1, 2001)]:
        kp = sched.select(v)["kp"]
        jump = abs(kp - prev_kp)
        max_jump = max(max_jump, jump)
        prev_kp = kp

    print(f"Maximum kp jump between consecutive samples: {max_jump:.6f}")
    assert max_jump < 0.01, f"Transition too abrupt: {max_jump}"
    print("Transition is smooth.")
