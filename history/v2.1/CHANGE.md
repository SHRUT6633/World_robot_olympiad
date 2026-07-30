# v2.1 — Turn Test

## What changed

With forward drive working reliably, the next step was turning. Our robot uses a four-wheel Ackermann steering configuration: two front wheels steer via a servo, two rear wheels drive. This is the same geometry as a car, and it means the inside and outside front wheels need to turn at different angles to avoid scrubbing. I knew this theoretically. I ignored it practically. That was a mistake.

I wrote `turn_test.py` to command the steering servo to various angles while driving slowly forward. The servo is connected to the ESP32 on GPIO 13, driven by the same MCPWM peripheral as the drive motor but on a different timer (timer 1). The servo expects a 50 Hz PWM signal with pulse width ranging from 1000 µs (full left) to 2000 µs (full right), centered at 1500 µs (straight).

The steering geometry I initially implemented was dead simple: set the servo angle proportional to the desired turn radius. Want to turn left? Set servo to 30 degrees. Want to turn right? Set servo to -30 degrees. Both front wheels turned by the same angle—like a go-kart, not a car.

The test procedure was: drive forward at 20% speed, turn the steering to 30 degrees, measure the turning radius. I marked a circle on the lab floor with chalk, drove the robot, and watched it carve a path that was way outside the calculated circle.

## Why it changed

The robot needs to navigate a line-following course with sharp turns. The WRO 2026 competition course has 90-degree and 180-degree turns with a minimum radius of about 0.5 meters. If the turning radius is too large, the robot will miss turns or hit the walls. Accurate turning is essential for the line-following task—the camera can only see so far ahead, and if the robot can't physically make the turn, the vision system can't compensate.

## Errors encountered

The turning radius error was huge. I calculated that a 30-degree steering angle should give a turning radius of about 0.6 meters (using the formula `R = wheelbase / tan(steering_angle)`). The actual measured radius was 0.95 meters—almost 60% larger.

The problem is Ackermann geometry. In a real car, the inside front wheel turns at a sharper angle than the outside front wheel. The wheels follow concentric circles around the same center point. If both wheels turn at the same angle, they fight each other—the inside wheel tries to turn tighter than the outside wheel allows, and the outside wheel tries to go straighter than the inside wheel wants. The result is tire scrubbing, increased resistance, and a larger effective turning radius because the robot's chassis resists the conflicting forces.

The steering linkage on our robot is a basic four-bar linkage (trapezoidal) that approximates Ackermann geometry, but only if the servo horn and steering arm lengths are correctly matched. I had the servo connected directly to the left wheel's kingpin, with a tie rod connecting the left and right wheels. The geometry depends on the distance between the kingpins, the length of the steering arms, and the angle of the steering arms relative to the axle.

I measured the actual geometry:
- Kingpin distance: 160 mm
- Steering arm length: 30 mm
- Steering arm angle (from perpendicular to axle): 15 degrees (toe-out)
- Wheelbase: 260 mm

Using these measurements, I calculated the required inside and outside wheel angles for a given turn radius:

```python
def ackermann_angles(radius, wheelbase, track_width):
    inside = atan(wheelbase / (radius - track_width/2))
    outside = atan(wheelbase / (radius + track_width/2))
    return degrees(inside), degrees(outside)
```

For a desired radius of 0.6 m:
- Inside wheel: atan(0.26 / (0.6 - 0.08)) = atan(0.26 / 0.52) = 26.6 degrees
- Outside wheel: atan(0.26 / (0.6 + 0.08)) = atan(0.26 / 0.68) = 20.9 degrees

The difference is 5.7 degrees—significant. With my original single-angle approach using 30 degrees (roughly the average), the inside wheel wasn't turning sharp enough and the outside wheel was turning too sharp, resulting in the measured 0.95 m radius.

I implemented a proper Ackermann steering model in `ackermann.py`. The servo controls one wheel directly via the steering arm, and the tie rod mechanically determines the other wheel's angle based on the four-bar linkage kinematics. The software now computes the correct servo position to achieve the desired average steering angle, accounting for the linkage geometry.

## Alternative approaches considered

1. **Four-wheel independent steering**: Each front wheel gets its own servo. This would give perfect Ackermann geometry but adds weight, cost, and complexity. Two servos need synchronization, and the mechanical mounting is tight.

2. **Skid-steer (tank drive)**: Drop the steering servo entirely and control left/right motor speeds independently. This would eliminate all the Ackermann complexity. But we already have the mechanical Ackermann linkage from the kit, and skid-steer is harder on the motors and tires.

3. **Caster steering**: Use a caster wheel like a shopping cart. Simple, but unstable at speed and hard to control precisely.

I stuck with Ackermann because the mechanical linkage is already built. The software fix costs nothing.

## Reasoning

The Ackermann fix moved the measured turning radius from 0.95 m to 0.62 m for a 30-degree commanded angle—much closer to the theoretical 0.6 m. The remaining 2 cm error is due to tire slip and linkage compliance. Good enough for now.

I also added a calibration routine: the robot now sweeps the servo from center to full lock in each direction while measuring the actual turning radius with a tape measure. This gives us a lookup table (servo PWM value → actual steering angle) that compensates for any nonlinearity in the servo or linkage.

The `turn_test.py` script now tests five angles (-30, -15, 0, 15, 30 degrees) at 20% speed, measuring and logging the actual turning radius for each. These calibration values feed into the `ackermann.py` module, which the rest of the system uses for all steering commands.
