# =============================================================================
# WRO 2026 - 4WS AWD Autonomous Robot
# File: simulations/theoretical_drivetrain.py
# Rev:  v0.1  |  Status: RESEARCH (pre-code)
# =============================================================================
# Pre-build theoretical simulation comparing FWD, RWD and AWD.
# Run: python simulations/theoretical_drivetrain.py
# No hardware required - pure torque and traction analysis.
# =============================================================================

import math

L = 0.300   # wheelbase (m)
W = 0.250   # track width (m)
M = 2.0     # robot mass (kg)
g = 9.81    # gravity
mu = 0.7    # tyre-road friction coefficient (rubber on vinyl)

print("=" * 60)
print("THEORETICAL: FWD vs RWD vs AWD Drivetrain Comparison")
print("=" * 60)
print(f"Mass: {M}kg  |  Wheelbase: {L*1000:.0f}mm  |  Track: {W*1000:.0f}mm")
print(f"Friction: mu={mu}  |  Max traction: {mu*M*g:.1f}N")
print("-" * 60)

# -- Torque Calculation --
T_motor = 0.12   # Nm (typical 370 DC motor)
GR = 9.6         # gear ratio (motor to wheel)
eta = 0.85       # drivetrain efficiency
T_wheel = T_motor * GR * eta

print(f"\n[Torque Budget]")
print(f"  Motor torque: {T_motor*1000:.0f} mNm")
print(f"  Gear ratio: {GR}:1")
print(f"  Efficiency: {eta*100:.0f}%")
print(f"  Wheel torque: {T_wheel*1000:.0f} mNm ({T_wheel:.2f} Nm)")

# -- Wheel Speed Differential --
print(f"\n[Wheel Speed Differential in Turns]")
print(f"{'R_turn':>8} | {'Ratio outer/inner':>18} | {'Binding?':>10}")
print("-" * 45)

for R_mm in [1000, 500, 400, 321, 250, 200]:
    R = R_mm / 1000
    ratio = (R + W/2) / (R - W/2)
    binding = "BIND" if ratio > 2.0 else "OK" if ratio < 1.5 else "WARN"
    print(f"{R_mm:>5}mm  | {ratio:>15.2f}x | {binding:>10}")

# -- Traction by Drivetrain --
print(f"\n[Traction Analysis]")
print(f"{'Config':>10} | {'Max accel':>10} | {'Max climb':>10} | {'Braking':>10} | {'Reverse':>10}")
print("-" * 55)

configs = [
    ("FWD", 0.45, 0.35, 0.30, 0.25),
    ("RWD", 0.55, 0.50, 0.50, 0.60),
    ("AWD", 0.70, 0.65, 0.65, 0.70),
]

for name, accel, climb, brake, rev in configs:
    accel_g = accel * g
    climb_deg = math.degrees(math.asin(climb))
    brake_g = brake * g
    rev_g = rev * g
    print(f"{name:>10} | {accel_g:>5.1f}m/s2 | {climb_deg:>5.1f}deg | {brake_g:>5.1f}m/s2 | {rev_g:>5.1f}m/s2")

# -- Power Distribution --
print(f"\n[Power Distribution Efficiency]")
P_motor = T_motor * (2 * math.pi * 15000 / 60)  # 15000 RPM no-load
print(f"  Motor power (no-load): {P_motor:.1f}W")

for name, mech_loss, slip_loss in [("FWD", 0.08, 0.12), ("RWD", 0.10, 0.10), ("AWD", 0.15, 0.05)]:
    eff_total = (1 - mech_loss) * (1 - slip_loss)
    P_delivered = P_motor * eff_total
    print(f"  {name}: mech_loss={mech_loss*100:.0f}% + slip_loss={slip_loss*100:.0f}% = {eff_total*100:.0f}% eff ({P_delivered:.1f}W)")

# -- Binding Torque Estimate --
print(f"\n[Binding Torque at R=321mm]")
R = 0.321
ratio = (R + W/2) / (R - W/2)
delta_omega = (ratio - 1)  # normalised speed difference
N_wheel = M * g / 4  # normal force per wheel
r_tire = 0.032  # tire radius (m)
T_bind = 0.3 * N_wheel * delta_omega * r_tire  # simplified binding model
print(f"  Speed ratio: {ratio:.2f}x")
print(f"  Normal force/wheel: {N_wheel:.1f}N")
print(f"  Binding torque: {T_bind*1000:.0f} mNm")
if T_bind < T_wheel:
    print(f"  Binding torque < wheel torque -> compliant coupling absorbs OK")
else:
    print(f"  Binding torque > wheel torque -> NEED DIFFERENTIAL")

print("\n" + "=" * 60)
print("CONCLUSION: AWD offers best traction and braking.")
print("Compliant coupling handles binding at low speed.")
print("AWD chosen as optimal drivetrain.")
print("=" * 60)
