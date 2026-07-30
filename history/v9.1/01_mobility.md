# 1. Mobility Management (4 pts)

## Vehicle Configuration
- **4-Wheel Steering (4WS)** with ONE steering servo via mechanical linkage (Rule 11.3 compliant)
- **All-Wheel Drive (AWD)** with ONE DC motor via chain/gear drivetrain (Rule 11.5 compliant)
- **File/line:** `pi/dynamics/kinematic_model.py:10` — KinematicModel class implements 4WS bicycle model
- **File/line:** `esp/main/l298n.c:23-25` — Single L298N channel drives all 4 wheels

## Steering Modes (config/surprise_rules.yaml)
| Mode | Front | Rear | Use Case | Code Location |
|------|-------|------|----------|--------------|
| SAME_PHASE | +delta | +delta | High-speed straights, gentle curves | `pi/dynamics/steering_modes.py:12-14` |
| OPPOSITE_PHASE | +delta | -delta | Tight turns, parking, narrow track | `pi/dynamics/steering_modes.py:15-18` |
| CRAB_WALK | +delta | +delta | Sideways parking, emergency dodge | `pi/dynamics/steering_modes.py:19-22` |

## Why 4WS?
- Reduces turning radius by ~50% vs 2WS (0.26m wheelbase -> 0.13m radius in opposite-phase)
- Enables crab-walk for parallel parking (criterion 1.8.2 = 15 pts)
- Switchable modes adapt to Surprise Rules without mechanical changes
- **File/line:** `pi/dynamics/steering_modes.py:30-33` — _turning_radius() computes radius from front/rear angles

## Key Files
- `pi/dynamics/steering_modes.py` — Angle computation for all 3 modes (34 lines)
- `pi/dynamics/kinematic_model.py` — 4WS bicycle model with mode support
- `pi/control/stanley.py` — Stanley lateral controller (40 lines)
- `pi/control/servo_pid.py` — Servo position PID closed-loop
- `esp/main/l298n.c` — Single-motor L298N driver (133 lines)
- `esp/main/servo_pwm.c` — Single-servo 4WS driver
