# v1.3 — Motor Spin Test

## Testing the L298N Motor Driver

The L298N dual H-bridge motor driver is responsible for driving our robot's two DC motors (left and right). Each H-bridge can handle up to 2A continuous current at 5-35V, with built-in flyback diodes for inductive load protection. We configured the L298N in PWM mode, where the ENA (enable) pin receives a PWM signal for speed control, and the IN1/IN2 pins determine direction.

Our test objective was straightforward: drive the motor forward for 2 seconds, stop for 1 second, then drive in reverse for 2 seconds. This test would confirm that the motor wiring is correct, the L298N is receiving power, and the GPIO control signals from the ESP32 are working.

## First Error: Motor Only Goes Forward, Not Reverse

When we ran the test, the motor spun forward correctly for 2 seconds, then stopped for 1 second, but when it was supposed to reverse, it remained stopped. The IN1 and IN2 pins were toggling correctly (we verified with a logic analyzer), so the direction control signals were working. The issue was on the ENA pin.

Initially, we had configured ENA as a simple digital GPIO output, setting it HIGH for on and LOW for off. This works for forward motion — when ENA is HIGH, the H-bridge is enabled and drives the motor according to the IN1/IN2 state. However, for reverse direction in an L298N, the H-bridge needs the enable pin to be modulated with a PWM signal. If ENA is just a static HIGH, the H-bridge drives the motor in whichever direction IN1/IN2 specify, but the transition from forward to reverse requires the ENA pin to toggle to allow the H-bridge to change the current direction through the motor windings.

The root cause was a misunderstanding of the L298N datasheet. The L298N H-bridge has two enable inputs (ENA for Motor A, ENB for Motor B) that are active-high enables. When ENA is HIGH, the outputs follow the IN1/IN2 inputs. When ENA is LOW, the outputs are disabled (motors coast). The datasheet recommends using PWM on the ENA pin for speed control, but it also states that ENA must be pulsed LOW between direction changes to allow the H-bridge to commutate. Without a LOW pulse on ENA between direction transitions, the H-bridge can latch in the previous state.

## The Fix: LEDC PWM on ENA

We reconfigured ENA as a PWM output using the ESP32's LEDC (LED Control) peripheral. The LEDC module can generate PWM signals with configurable frequency and duty cycle independently of the CPU, making it ideal for motor control. We configured a 1000Hz PWM signal with 8-bit resolution (0-255 duty cycle).

```c
ledc_timer_config_t timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT, .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000, .clk_cfg = LEDC_AUTO_CLK };
ledc_timer_config(&timer);
ledc_channel_config_t chan = { .gpio_num = ENA,
    .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0, .duty = 0 };
ledc_channel_config(&chan);
```

With this configuration, we could set the duty cycle to control the motor speed. Setting duty to 200 (out of 255) gave approximately 78% speed. To change direction, we set the duty to 0 (effectively disabling the H-bridge), changed IN1/IN2, then ramped the duty back up. The PWM signal on ENA ensures that the H-bridge's output transistors switch cleanly between forward and reverse states.

## Alternative: L9110S Motor Driver

We considered replacing the L298N with the L9110S dual motor driver. The L9110S has a different control scheme: it has two input pins per motor (IA and IB) that control both direction and speed. To go forward, IA is PWM and IB is LOW. To go reverse, IB is PWM and IA is LOW. To stop, both are LOW. The L9110S does not require a separate enable pin, which simplifies the wiring and avoids the ENA PWM issue we encountered.

However, we decided to stick with the L298N for two reasons. First, we had already purchased and wired the L298N. Replacing it would mean rewiring and redesigning the motor mount plate. Second, the L298N is more robust: it handles 2A continuous per channel (vs. 800mA for L9110S), and our motors draw up to 1.5A under load. The L9110S would be operating at its limit, risking thermal shutdown during the competition.

## Learned: Read the Datasheet BEFORE Wiring

The most important lesson from v1.3 is to read the component datasheet thoroughly before connecting it. Our assumption that ENA could be a simple digital output was based on experience with other H-bridge drivers (like the TB6612FNG), which have different control logic. The L298N datasheet explicitly states that PWM on the enable pins is recommended for speed control, and that direction changes require a brief disable period.

We now maintain a hardware reference sheet in `docs/hardware_pinout.md` that documents each component's control requirements. This sheet includes the L298N's truth table, the servo's pulse width range, and the sensor's I2C addresses. Having this reference prevents repeating the same mistake with other components.

## Motor Current and Power Supply

During the forward/reverse test, we measured the motor current with a clamp meter. At 78% duty cycle, the motor drew approximately 1.2A with no load. Under stall conditions (motor shaft held stationary), the current rose to 2.1A within 2 seconds. The L298N's thermal protection kicked in after 5 seconds of stall, reducing output power. For competition, we need to ensure that the robot never stalls for more than 1 second, or we risk damaging the motor driver.

The 5V power supply from the Pi's GPIO header was insufficient for the motors. We now use a separate 5V/5A buck converter powered from the 11.1V LiPo battery. The L298N's logic supply (5V input) is still powered from the Pi, but the motor supply (12V input) comes from the buck converter. This separation prevents motor noise from coupling into the Pi's power rail.

## Motor Encoder Considerations

Our current motor test does not include encoders. The motors we selected are simple DC gearmotors without encoder output. This means we cannot measure the actual wheel speed or position — we can only command a PWM duty cycle and hope the motor responds appropriately. For line following, this open-loop control is acceptable because the camera provides visual feedback of the robot's position relative to the line. However, for precise distance tracking (required for the WRO obstacle course), we may need to add encoders in a future hardware revision. The alternative is to use the ToF sensors to measure distance traveled by tracking the time-of-flight readings as the robot moves past walls and obstacles. This visual odometry approach is less accurate than encoders but does not require additional hardware.

## Thermal Testing of L298N

We ran the motor forward at full speed for 60 seconds and measured the L298N heatsink temperature with a thermocouple. The temperature rose from 25°C (ambient) to 55°C after 60 seconds, then stabilized at approximately 58°C after 3 minutes. The L298N's thermal shutdown threshold is 170°C, so we are well within safe operating limits. However, the heatsink became noticeably hot to the touch (above 50°C). For the competition, we will add a small 5V fan (40mm, 0.1A) blowing across the L298N heatsink to improve airflow inside the robot chassis. The fan will be controlled by a MOSFET switch on GPIO 26, turning on only when the motors are active and the temperature exceeds 45°C. This active cooling should keep the L298N below 40°C even during extended operation.
