# v8.1 — Opposite-Phase Steering Implementation

## What Changed

I implemented opposite-phase steering today. In this mode, the front wheels turn one direction and the rear wheels turn the opposite direction, causing the robot to rotate around its center point. This is incredibly useful for tight maneuvers — the turning radius drops to 0.5 meters, which is almost half of what same-phase steering can do.

The new module is `steer_opposite.py`. The geometry is: front wheels at angle +θ, rear wheels at angle -θ. The turning radius formula changes because the center of rotation is now at the robot's center instead of somewhere along the wheelbase extension. I derived: `θ = atan(wheelbase / (2 * R))` — same formula actually, but the interpretation is different. For same-phase, both front and rear are +θ. For opposite-phase, front is +θ, rear is -θ.

## Errors Encountered

The first test was alarming. When I engaged opposite-phase steering at full speed (0.8 m/s), the robot instantly spun around its center like a top. The IMU went haywire:

```
[IMU] WARN: Gyro Z-axis reading: 120 deg/s — rate saturation detected
[CONTROL] ERROR: Yaw error = 180.3 deg — controller attempting to correct
[CONTROL] ERROR: Integral windup detected — resetting I term
[SAFETY] EMERGENCY STOP: Angular velocity exceeds 90 deg/s threshold
```

The problem is that opposite-phase steering is fundamentally different from same-phase. In same-phase, the robot describes a large arc and the yaw changes slowly. In opposite-phase, the robot rotates around its center, so the yaw changes rapidly even at low speeds. The PID controller was tuned for same-phase dynamics and immediately went into integral windup trying to counteract what it thought was a disturbance.

## The Fix

I added a special handling mode: when opposite-phase steering is active, the target speed is capped at 0.3 m/s. This keeps the angular velocity below 30 deg/s, which the IMU can handle and the controller doesn't fight.

The fix involved overriding the speed command in the steering module:
```python
if mode == "opposite_phase":
    speed_cmd = min(speed_cmd, 0.3)  # m/s
```

I also added a mode flag to the controller so it knows not to apply yaw correction during opposite-phase maneuvers. The robot's rotation is intentional, not a disturbance.

## Alternatives Considered

1. **Re-tuning the PID controller**: I could have added a separate PID gain set for opposite-phase mode. The P gain would need to be much lower (around 0.2 instead of 1.5) to prevent windup. But this would make the controller sluggish for the brief transition between modes. Mode-switching PID gains can cause nasty bumps in the control output.

2. **Feed-forward gyro compensation**: Instead of capping speed, I could feed the expected rotation rate into the controller so it doesn't treat it as an error. This is actually the "correct" solution, but it requires a accurate model of the vehicle dynamics. I don't have that yet — we'd need system identification tests. The speed cap is a pragmatic stopgap.

3. **Gradual angle ramp**: Instead of instantly applying the full steering angle, I could ramp it over 500ms. This would give the controller time to adapt. I tested this and it works, but it adds latency to the steering response. In a competition, you need instant response when avoiding obstacles.

## Testing

After the speed cap fix:
- Turning radius: 0.48-0.52m (meets 0.5m target)
- Angular velocity: 25 deg/s at 0.3 m/s
- No IMU saturation
- No controller windup
- Robot completes 180° turn in ~2.4 seconds

The speed reduction is acceptable because opposite-phase steering is only used in the parking zone where precision matters more than speed.

## Lessons Learned

Opposite-phase steering is powerful but dangerous. The high angular velocity surprised me because I was thinking in terms of linear speed, not rotational speed. I should add angular velocity limits to the safety monitor in a future version. Also, the controller architecture needs to be mode-aware — different steering modes have fundamentally different dynamics and the controller shouldn't fight the intended behavior.
