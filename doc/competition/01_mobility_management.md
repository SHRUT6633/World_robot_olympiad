# 1. Mobility Management (4 pts)

## Vehicle Configuration
- **4-Wheel Steering (4WS)** with ONE steering servo via mechanical linkage
- **All-Wheel Drive (AWD)** with ONE DC motor via chain/gear drivetrain
- Compliant with WRO Rule 11.3 (one steering actuator) and Rule 11.5 (no electronic differential)

## Steering Modes (config/surprise_rules.yaml)
| Mode | Front | Rear | Use Case |
|------|-------|------|----------|
| SAME_PHASE | +delta | +delta | High-speed straights, gentle curves |
| OPPOSITE_PHASE | +delta | -delta | Tight turns, parking, narrow track |
| CRAB_WALK | +delta | +delta | Sideways parking, emergency dodge |

## Why 4WS?
- Reduces turning radius by ~50% vs 2WS (0.26m wheelbase → 0.13m radius in opposite-phase)
- Enables crab-walk for parallel parking (criterion 1.8.2 = 15 pts)
- Switchable modes adapt to Surprise Rules without mechanical changes

## Files
- `pi/dynamics/steering_modes.py` — Angle computation for all 3 modes
- `pi/dynamics/kinematic_model.py` — 4WS bicycle model with mode support
- `pi/control/stanley.py` — Stanley lateral controller
- `pi/control/servo_pid.py` — Servo position PID
- `esp/main/l298n.c` — Single-motor L298N driver
- `esp/main/servo.c` — Single-servo 4WS driver
