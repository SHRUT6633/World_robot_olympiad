# v1.4 — Servo Calibration

## Testing the Steering Servo

Our robot uses a standard micro servo (SG90 or equivalent) for steering. The servo rotates the front wheel assembly left and right, controlled by a 50Hz PWM signal with pulse widths ranging from 500μs to 2500μs. The servo's neutral position (wheels straight) is at approximately 1500μs pulse width.

The goal of this test was to sweep the servo through its full claimed range of motion, measure the actual wheel deflection at each pulse width, and build a calibration table mapping pulse width to steering angle. This calibration is critical for the control system: when the high-level logic says "turn 15 degrees left," the low-level servo controller needs to know exactly which pulse width corresponds to that angle.

## First Error: Servo Jitters at Extreme Angles

As we swept the servo from 500μs to 2500μs in 50μs steps, we observed smooth motion within a central range but severe jittering and buzzing at the extremes. The jittering was accompanied by an audible high-pitched whine from the servo motor. At pulse widths below 700μs or above 2300μs, the servo would oscillate rapidly between two positions instead of holding a steady angle.

The root cause was that the SG90 servo has a limited mechanical travel range. While the datasheet claims 180 degrees of rotation (500-2500μs maps to -90 to +90 degrees), the actual mechanical stops inside the servo limit movement to approximately 120 degrees (-60 to +60). When we commanded pulse widths beyond the mechanical stops, the servo's internal potentiometer (which measures the output shaft position) could not provide valid feedback, causing the control circuit to oscillate as it tried to reach an unattainable position.

## The Fix: Software Range Limiting

We limited the software range to ±30 degrees (approximately 1000-2000μs). This is well within the servo's mechanical limits and provides enough steering angle for our track. The WRO track has gentle curves with radii of approximately 50cm, which require at most 20 degrees of steering at our target speed of 0.5 m/s. The ±30 degree range gives us a safety margin for tighter turns while remaining within the servo's stable operating region.

The calibration table maps pulse width to steering angle linearly. We measured the actual wheel deflection at various pulse widths using a protractor and built the following mapping:

| Pulse (μs) | Angle (deg) |
|-------------|-------------|
| 1000        | -30         |
| 1250        | -15         |
| 1500        | 0           |
| 1750        | +15         |
| 2000        | +30         |

The relationship is approximately linear: each degree requires about 16.7μs change in pulse width. Our code uses a 14-bit PWM resolution (16384 steps over a 20ms period), so the pulse width formula is:

```c
uint32_t pulse = 1638 + (deg + 30) * 27;
```

The 1638 value corresponds to a 1500μs pulse (1638 / 16384 * 20ms ≈ 2.0ms, adjusted for the 14-bit range to match 1500μs). Each degree adds 27 counts, giving approximately 16.7μs per degree at 14-bit resolution.

## Alternatives Considered

We considered three alternatives before settling on software range limiting.

Replace with a better servo: A high-quality servo like the MG996R has metal gears and a wider mechanical range (180 degrees). However, the MG996R is larger and heavier (55g vs 9g for SG90), requiring a redesign of the steering mount. It also draws more current (up to 2A under stall), which would stress our 5V regulator. Given that the SG90 meets our requirements with the limited range, replacement was unnecessary.

Gear reduction: Adding a gear train between the servo and the steering linkage would reduce the angular range while increasing torque. However, gears introduce backlash (dead zone where the gears don't engage), which would reduce steering precision. For a robot that needs to follow a line accurately, backlash of even 1-2 degrees could cause oscillation in the control loop.

Different servo control method: Instead of standard PWM, some servos support I2C or serial control. These servos offer higher precision and feedback, but they are more expensive and require different wiring. Our existing PWM-based approach is simpler and sufficient for the task.

## Learned: Respect Mechanical Limits

The servo jitter taught us an important lesson about respecting component specifications. Datasheet claims are often optimistic, especially for cheap components like the SG90 servo. The "180 degree" range is a theoretical maximum assuming perfect mechanical alignment and no load. In practice, the servo's output shaft hits mechanical stops at about 120 degrees, and the internal potentiometer linearity degrades near the extremes.

We now apply a 20% safety margin to all component specifications. If a servo claims 180 degrees, we only use 120. If a motor driver claims 2A, we only draw 1.6A. This derating practice improves reliability at the cost of reduced capability. For a competition robot that must complete its task on the first attempt, reliability is more important than peak performance.

## PWM Frequency and Resolution Tradeoffs

The servo expects a 50Hz PWM signal (20ms period). We configured the LEDC timer with 14-bit resolution, giving us 16384 duty steps over the 20ms period. Each step corresponds to approximately 1.22μs (20ms / 16384). With a pulse range of 1000μs to 2000μs (our ±30° range), we use approximately 819 steps out of 16384 available. The 14-bit resolution gives us about 0.073 degrees per step, which is far finer than the servo's mechanical resolution (approximately 0.5 degrees due to gear backlash). We could have used a lower resolution (e.g., 10-bit, 1024 steps) and still achieved sufficient precision, but the 14-bit setting does not consume any additional CPU resources because the LEDC peripheral handles the PWM generation entirely in hardware. Higher resolution also gives us headroom if we need to extend the pulse range in the future.

## Mechanical Considerations

The servo is mounted on a 3D-printed bracket that connects to the steering linkage via a pushrod. We measured the mechanical slack in the linkage: approximately 1 degree of free play before the wheels respond to servo movement. This backlash is caused by the clearance between the pushrod ball joints and their sockets. Over time, this backlash will increase as the plastic components wear. We will monitor the backlash weekly and replace the pushrod if it exceeds 3 degrees. During the competition, we will apply a thin layer of lithium grease to the ball joints to reduce friction and wear. The steering pivot also has a bronze bushing that we lubricate with silicone grease; this bushing showed no measurable wear after 50 hours of testing.

## Calibration Procedure

The calibration test is run every time the robot is assembled at a new venue. Temperature changes can affect the servo's neutral position, and mechanical wear over time changes the steering linkage geometry. The calibration procedure is:

1. Place robot on blocks so wheels are off the ground
2. Run servo_sweep() which commands angles from -30 to +30 in 5-degree steps
3. Measure actual wheel angle with a digital protractor
4. Adjust the calibration table if measured angles differ from expected
5. Store adjusted calibration values in non-volatile storage (NVS) on the ESP32

The calibration is stored as a set of (pulse_width, angle) pairs that the servo controller interpolates between. This allows us to correct any nonlinearity in the servo's response.
