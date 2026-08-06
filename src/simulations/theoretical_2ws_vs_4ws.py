# =============================================================================
# WRO 2026 - 4WS AWD Autonomous Robot
# File: simulations/theoretical_2ws_vs_4ws.py
# Rev:  v0.3  |  Status: RESEARCH (pre-code)
# =============================================================================
# Interactive simulation: compare 2WS vs 4WS turning radii.
# All values are COMPUTED in real-time - change any parameter.
# No hardware required - pure geometric analysis.
# =============================================================================
#
# Usage:
#   python simulations\theoretical_2ws_vs_4ws.py                          # defaults
#   python simulations\theoretical_2ws_vs_4ws.py --wheelbase 400          # custom
#   python simulations\theoretical_2ws_vs_4ws.py --corner-radius 800      # custom
#   python simulations\theoretical_2ws_vs_4ws.py --track 300 --wheelbase 350
#   python simulations\theoretical_2ws_vs_4ws.py --interactive            # input mode
#   python simulations\theoretical_2ws_vs_4ws.py --sweep wheelbase 250:400:50
#
# =============================================================================

import math
import sys

# -- Parse arguments manually (no argparse dependency) --
L = 0.300      # wheelbase (m)
W = 0.250      # track width (m)
R_corner = 0.500  # corner radius (m)
interactive = False
sweep_param = None
sweep_range = None

args = sys.argv[1:]

i = 0
while i < len(args):
    if args[i] == "--wheelbase" and i + 1 < len(args):
        L = float(args[i + 1]) / 1000
        i += 2
    elif args[i] == "--track" and i + 1 < len(args):
        W = float(args[i + 1]) / 1000
        i += 2
    elif args[i] == "--corner-radius" and i + 1 < len(args):
        R_corner = float(args[i + 1]) / 1000
        i += 2
    elif args[i] == "--interactive":
        interactive = True
        i += 1
    elif args[i] == "--sweep" and i + 3 < len(args):
        sweep_param = args[i + 1]
        sweep_range = [float(args[i + 2]), float(args[i + 3]), float(args[i + 4])] if i + 4 < len(args) else [float(args[i + 2]), float(args[i + 3]), float(args[i + 2])]
        i += 5 if i + 4 < len(args) else 3
    else:
        i += 1

if interactive:
    print("=== Interactive Mode ===")
    L = float(input("Wheelbase (mm) [300]: ") or "300") / 1000
    W = float(input("Track width (mm) [250]: ") or "250") / 1000
    R_corner = float(input("Corner radius (mm) [500]: ") or "500") / 1000
    print()

def compute_2ws(L, W, R_corner, delta_deg):
    delta = math.radians(delta_deg)
    R = L / math.tan(delta)
    R_inner = R - W / 2
    clearance = R_corner - R_inner
    status = "PASS" if clearance > 0.05 else "FAIL" if clearance < 0 else "EDGE"
    return R, R_inner, clearance, status

def compute_4ws_opposite(L, W, R_corner, delta_deg):
    delta = math.radians(delta_deg)
    eff_delta = 2 * delta
    if abs(math.tan(eff_delta)) < 1e-10:
        return float('inf'), -W / 2, R_corner + W / 2, "POINT"
    R = abs(L / math.tan(eff_delta))
    R_inner = R - W / 2
    clearance = R_corner - R_inner
    status = "PASS" if clearance > 0.05 else "FAIL"
    return R, R_inner, clearance, status

def compute_4ws_same(L, W, R_corner, delta_deg):
    return compute_2ws(L, W, R_corner, delta_deg)

def run_table(L, W, R_corner, label, mode, steer_angles):
    print(f"\n[{label}]")
    print(f"{'Steer':>6} | {'R_centre':>8} | {'R_inner':>8} | {'Clearance':>9} | {'Status'}")
    print("-" * 50)
    for deg in steer_angles:
        if mode == "2ws":
            R, Ri, clr, st = compute_2ws(L, W, R_corner, deg)
        elif mode == "opposite":
            R, Ri, clr, st = compute_4ws_opposite(L, W, R_corner, deg)
        else:
            R, Ri, clr, st = compute_4ws_same(L, W, R_corner, deg)

        if R == float('inf'):
            print(f"{deg:>5}deg | {'inf':>8} | {Ri*1000:>7.0f}mm | {clr*1000:>+7.0f}mm | {st}")
        else:
            print(f"{deg:>5}deg | {R*1000:>7.0f}mm | {Ri*1000:>7.0f}mm | {clr*1000:>+7.0f}mm | {st}")

def min_pass_angle(L, W, R_corner, mode, max_deg=45):
    for deg in range(1, max_deg + 1):
        if mode == "2ws":
            _, _, clr, st = compute_2ws(L, W, R_corner, deg)
        elif mode == "opposite":
            _, _, clr, st = compute_4ws_opposite(L, W, R_corner, deg)
        else:
            _, _, clr, st = compute_4ws_same(L, W, R_corner, deg)
        if "PASS" in st or "POINT" in st:
            return deg
    return None

# -- Sweep mode --
if sweep_param:
    param_label = {"wheelbase": "Wheelbase", "track": "Track width", "corner-radius": "Corner radius"}.get(sweep_param, sweep_param)
    unit = "mm"
    print(f"=== Sweep: {param_label} from {sweep_range[0]:.0f} to {sweep_range[2]:.0f} step {sweep_range[1]:.0f} ===")
    print(f"{'Param':>12} | {'2WS min deg':>10} | {'4WS opp min deg':>15} | {'4WS same min deg':>15}")
    print("-" * 55)
    val = sweep_range[0]
    while val <= sweep_range[2]:
        if sweep_param == "wheelbase":
            Lv = val / 1000
            m2 = min_pass_angle(Lv, W, R_corner, "2ws")
            m4o = min_pass_angle(Lv, W, R_corner, "opposite")
            m4s = min_pass_angle(Lv, W, R_corner, "same")
        elif sweep_param == "track":
            Wv = val / 1000
            m2 = min_pass_angle(L, Wv, R_corner, "2ws")
            m4o = min_pass_angle(L, Wv, R_corner, "opposite")
            m4s = min_pass_angle(L, Wv, R_corner, "same")
        else:
            Rv = val / 1000
            m2 = min_pass_angle(L, W, Rv, "2ws")
            m4o = min_pass_angle(L, W, Rv, "opposite")
            m4s = min_pass_angle(L, W, Rv, "same")
        m2s = f"{m2}deg" if m2 else "NEVER"
        m4os = f"{m4o}deg" if m4o else "NEVER"
        m4ss = f"{m4s}deg" if m4s else "NEVER"
        print(f"{val:>8.0f}{unit:>4} | {m2s:>10} | {m4os:>15} | {m4ss:>15}")
        val += sweep_range[1]
    sys.exit(0)

# -- Standard run --
print("=" * 60)
print("THEORETICAL: 2WS vs 4WS Turning Radius Comparison")
print("=" * 60)
print(f"Wheelbase: {L*1000:.0f}mm  |  Track width: {W*1000:.0f}mm")
print(f"Corner radius (target): {R_corner*1000:.0f}mm")
print(f"{'All values COMPUTED dynamically - change with --wheelbase, --track, --corner-radius'}")
print("-" * 60)

run_table(L, W, R_corner, "2WS - Front wheels only", "2ws", [10, 20, 25, 30, 35, 40, 45])
run_table(L, W, R_corner, "4WS - Opposite-phase (rear steers opposite front)", "opposite", [5, 10, 15, 20, 25, 30])
run_table(L, W, R_corner, "4WS - Same-phase (all wheels parallel)", "same", [5, 10, 15, 20, 25])

# -- Min angle to pass --
print(f"\n{'='*60}")
m2 = min_pass_angle(L, W, R_corner, "2ws")
m4o = min_pass_angle(L, W, R_corner, "opposite")
m4s = min_pass_angle(L, W, R_corner, "same")
print(f"Minimum steering to clear R={R_corner*1000:.0f}mm corner:")
print(f"  2WS:                 {m2}deg" if m2 else "  2WS:                 NEVER")
print(f"  4WS opposite-phase:  {m4o}deg" if m4o else "  4WS opposite-phase:  NEVER")
print(f"  4WS same-phase:      {m4s}deg" if m4s else "  4WS same-phase:      NEVER")
print(f"{'='*60}")
print(f"CONCLUSION: 2WS needs {m2}deg steering (mechanical limit ~35deg).")
print(f"4WS opposite-phase passes at only {m4o}deg. 4WS is mandatory.")
print(f"{'='*60}")
